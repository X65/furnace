/**
 * Furnace Tracker - multi-system chiptune tracker
 * Copyright (C) 2021-2026 tildearrow and contributors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "sgu.h"
#include "../engine.h"
#include "../../ta-log.h"
#include "sound/sgu.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

// SGU-1 replayer notes:
// - Entry points: init/reset/tick/acquire wire the core to Furnace's timing/output path.
// - FM vs PCM: commitState picks FM (ESFM-style ops) or PCM (Amiga sample path via SoundUnit-style regs).
// - SID-like pitch: NOTE_FREQUENCY gives freq16 @ 1 MHz; calcFreq keeps freq16 with CHIP_FREQBASE, then write SGU FREQ regs.
// - OPN envelope: AR/DR/SR are 5-bit; MSBs live in op reg7, SR lives in reg4 (applyOpRegs).
// - RING/SYNC: per-operator bits in reg6; masks map op0..3 to the previous operator (op0 uses op3).
// - Global regs: SoundUnit-style flags0/flags1; KEY-ON is flags0 bit0; WAVE bits live per-operator.

#define SGU_REG_POOL_SIZE (SGU_CHNS * SGU_REGS_PER_CH)
#define rWrite(a,v) if (!skipRegisterWrites) {writes.push(QueuedWrite(a,v)); if (dumpWrites) {addWrite(a,v);} if ((a) < SGU_REG_POOL_SIZE) {reinterpret_cast<unsigned char*>(chip.chan)[(a)]=(v);} }

static constexpr int SGU_CH_BASE = SGU_OP_PER_CH * SGU_OP_REGS;

#define opWrite(c,o,a,v) rWrite(((c) * SGU_REGS_PER_CH) + ((o) * SGU_OP_REGS) + (a), (v))
#define chWrite(c,a,v)   rWrite(((c) * SGU_REGS_PER_CH) + SGU_CH_BASE + (a), (v))

#define CHIP_FREQBASE 524288

static const char* regCheatSheetSGU[]={
  "CHx_OPy_R0 [7 TRM][6 VIB][5 FIX][3:0 MUL]", "00+x*40+y*08",
  "CHx_OPy_R1 [7:6 KSL][5:0 TL]", "01+x*40+y*08",
  "CHx_OPy_R2 [7:4 AR][3:0 DR]", "02+x*40+y*08",
  "CHx_OPy_R3 [7:4 SL][3:0 RR]", "03+x*40+y*08",
  "CHx_OPy_R4 [7:5 DT][4:0 SR]", "04+x*40+y*08",
  "CHx_OPy_R5 [7:5 DELAY][4:3 KSR][2:0 WPAR]", "05+x*40+y*08",
  "CHx_OPy_R6 [7 TRMD][6 VIBD][5 SYNC][4 RING][3:1 MOD][0 TLmsb]", "06+x*40+y*08",
  "CHx_OPy_R7 [7:5 OUT][4 ARmsb][3 DRmsb][2:0 WAVE]", "07+x*40+y*08",

  "CHx_FREQ_L", "20+x*40",
  "CHx_FREQ_H", "21+x*40",
  "CHx_VOL", "22+x*40",
  "CHx_PAN", "23+x*40",
  "CHx_FLAGS0", "24+x*40",
  "CHx_FLAGS1", "25+x*40",
  "CHx_CUTOFF_L", "26+x*40",
  "CHx_CUTOFF_H", "27+x*40",
  "CHx_DUTY", "28+x*40",
  "CHx_RESON", "29+x*40",
  "CHx_PCM_POS_L", "2A+x*40",
  "CHx_PCM_POS_H", "2B+x*40",
  "CHx_PCM_END_L", "2C+x*40",
  "CHx_PCM_END_H", "2D+x*40",
  "CHx_PCM_RST_L", "2E+x*40",
  "CHx_PCM_RST_H", "2F+x*40",
  "CHx_SWFREQ_SPD_L", "30+x*40",
  "CHx_SWFREQ_SPD_H", "31+x*40",
  "CHx_SWFREQ_AMT", "32+x*40",
  "CHx_SWFREQ_BND", "33+x*40",
  "CHx_SWVOL_SPD_L", "34+x*40",
  "CHx_SWVOL_SPD_H", "35+x*40",
  "CHx_SWVOL_AMT", "36+x*40",
  "CHx_SWVOL_BND", "37+x*40",
  "CHx_SWCUT_SPD_L", "38+x*40",
  "CHx_SWCUT_SPD_H", "39+x*40",
  "CHx_SWCUT_AMT", "3A+x*40",
  "CHx_SWCUT_BND", "3B+x*40",
  "CHx_RESTIMER_L", "3C+x*40",
  "CHx_RESTIMER_H", "3D+x*40",
  "CHx_SPECIAL1", "3E+x*40",
  "CHx_SPECIAL2", "3F+x*40",
  NULL
};

const char** DivPlatformSGU::getRegisterSheet() {
  return regCheatSheetSGU;
}

void DivPlatformSGU::acquire(short** buf, size_t len) {
  thread_local int32_t o[2];
  for (int i=0; i<SGU_CHNS; i++) {
    oscBuf[i]->begin(len);
  }
  for (size_t h=0; h<len; h++) {
    if (!writes.empty()) {
      QueuedWrite& w=writes.front();
      SGU_Write(&chip,w.addr,w.val);
      writes.pop();
    }

    SGU_NextSample(&chip,&o[0],&o[1]);
    for (int c=0; c<SGU_CHNS; c++) {
      oscBuf[c]->putSample(h,SGU_GetSample(&chip,c));
    }

    buf[0][h] = (short)CLAMP(o[0], -32768, 32767);
    buf[1][h] = (short)CLAMP(o[1], -32768, 32767);
  }
  for (int i=0; i<SGU_CHNS; i++) {
    oscBuf[i]->end(len);
  }
}

void DivPlatformSGU::acquireDirect(blip_buffer_t** bb, size_t len) {
  thread_local int32_t o[2];
  unsigned int sharedNeedlePos=oscBuf[0]->needle;
  for (int i=0; i<SGU_CHNS; i++) {
    oscBuf[i]->begin(len);
  }
  for (size_t h=0; h<len; h++) {
    if (!writes.empty()) {
      QueuedWrite& w=writes.front();
      SGU_Write(&chip,w.addr,w.val);
      writes.pop();
    }

    SGU_NextSample(&chip,&o[0],&o[1]);
    const unsigned int shiftedNeedlePos=sharedNeedlePos>>OSCBUF_PREC;
    for (int c=0; c<SGU_CHNS; c++) {
      putSampleIKnowWhatIAmDoing(oscBuf[c],shiftedNeedlePos,SGU_GetSample(&chip,c));
    }
    sharedNeedlePos+=oscBuf[0]->rateMul;

    if (o[0]!=oldOut[0]) {
      blip_add_delta(bb[0],h,o[0]-oldOut[0]);
      oldOut[0]=o[0];
    }
    if (o[1]!=oldOut[1]) {
      blip_add_delta(bb[1],h,o[1]-oldOut[1]);
      oldOut[1]=o[1];
    }
  }
  for (int i=0; i<SGU_CHNS; i++) {
    oscBuf[i]->end(len);
  }
}

void DivPlatformSGU::applyOpRegs(int ch, int o, const DivInstrumentFM::Operator& op, const DivInstrumentESFM::Operator& opE) {
  const unsigned char tl = op.tl & 0x7f;  // 7-bit TL
  const unsigned char ar = op.ar & 0x1f;  // 5-bit AR
  const unsigned char dr = op.dr & 0x1f;  // 5-bit DR

  // R0: [7]TRM [6]VIB [5]FIX [4]--- [3:0]MUL
  const unsigned char reg0 = (op.am ? SGU_OP0_TRM_BIT : 0)
    | (op.vib ? SGU_OP0_VIB_BIT : 0)
    | (opE.fixed ? SGU_OP0_FIX_BIT : 0)
    | (op.mult & SGU_OP0_MUL_MASK);

  // R1: [7:6]KSL [5:0]TL_lo6
  const unsigned char reg1 = (op.ksl << SGU_OP1_KSL_SHIFT)
    | (tl & SGU_OP1_TL_MASK);

  // R2: [7:4]AR_lo4 [3:0]DR_lo4
  const unsigned char reg2 = ((ar & 0x0f) << SGU_OP2_AR_SHIFT)
    | (dr & SGU_OP2_DR_MASK);

  // R3: [7:4]SL [3:0]RR
  const unsigned char reg3 = (op.sl << SGU_OP3_SL_SHIFT)
    | (op.rr & SGU_OP3_RR_MASK);

  // R4: [7:5]DT [4:0]SR (SR uses OPN-style 5-bit value from d2r)
  const unsigned char reg4 = (op.dt << SGU_OP4_DT_SHIFT)
    | (op.d2r & SGU_OP4_SR_MASK);

  // R5: [7:5]DELAY [4:3]KSR [2:0]WPAR
  const unsigned char wpar = 0; // SGU WPAR not yet exposed via Furnace macros
  const unsigned char reg5 = (opE.delay << SGU_OP5_DELAY_SHIFT)
    | (op.rs << SGU_OP5_KSR_SHIFT)
    | (wpar & SGU_OP5_WPAR_MASK);

  // R6: [7]TRMD [6]VIBD [5]SYNC [4]RING [3:1]MOD [0]TL_msb
  // Note: SYNC and RING not yet exposed via Furnace macros
  const unsigned char reg6 = (op.dam ? SGU_OP6_TRMD_BIT : 0)
    | (op.dvb ? SGU_OP6_VIBD_BIT : 0)
    | (opE.modIn << SGU_OP6_MOD_SHIFT)
    | ((tl >> SGU_OP1_TL_OFFSET) & SGU_OP6_TL_MSB_BIT);

  // R7: [7:5]OUT [4]AR_msb [3]DR_msb [2:0]WAVE
  const unsigned char outLvl = op.enable ? (opE.outLvl & 0x07) : 0;
  const unsigned char reg7 = (outLvl << SGU_OP7_OUT_SHIFT)
    | ((ar & 0x10) ? SGU_OP7_AR_MSB_BIT : 0)
    | ((dr & 0x10) ? SGU_OP7_DR_MSB_BIT : 0)
    | (op.ws & SGU_OP7_WAVE_MASK);

  opWrite(ch, o, 0x00, reg0);
  opWrite(ch, o, 0x01, reg1);
  opWrite(ch, o, 0x02, reg2);
  opWrite(ch, o, 0x03, reg3);
  opWrite(ch, o, 0x04, reg4);
  opWrite(ch, o, 0x05, reg5);
  opWrite(ch, o, 0x06, reg6);
  opWrite(ch, o, 0x07, reg7);
}

static void setEsfmRouting(DivInstrumentESFM& esfm,
    const unsigned char modIn[SGU_OP_PER_CH],
    const unsigned char outLvl[SGU_OP_PER_CH]) {
  for (int o=0; o<SGU_OP_PER_CH; o++) {
    esfm.op[o].modIn=modIn[o];
    esfm.op[o].outLvl=outLvl[o];
  }
}

// Convert C64 waveform to SGU waveform
static unsigned char sguC64Wave(const DivInstrumentC64& c64, bool periodicNoise) {
  // C64 wave bits: 4=noise, 2=pulse, 1=saw, 0=tri (combined waveforms possible)
  if (c64.noiseOn) {
    return periodicNoise ? SGU_WAVE_PERIODIC_NOISE : SGU_WAVE_NOISE;
  }
  if (c64.pulseOn) return SGU_WAVE_PULSE;
  if (c64.sawOn) return SGU_WAVE_SAWTOOTH;
  if (c64.triOn) return SGU_WAVE_TRIANGLE;
  return SGU_WAVE_PULSE; // default
}

void DivPlatformSGU::writeControl(int ch) {
  // FLAGS0: [7:4]CONTROL [3]PCM [2:1]--- [0]GATE
  chWrite(ch, SGU1_CHN_FLAGS0,
    ((chan[ch].active && chan[ch].gate) ? SGU1_FLAGS0_CTL_GATE : 0)
    | (chan[ch].pcm << SGU1_FLAGS0_PCM_SHIFT)
    | (chan[ch].control << SGU1_FLAGS0_CONTROL_SHIFT));
}

void DivPlatformSGU::writeControlUpper(int ch) {
  // FLAGS1: [6]CUT_SWEEP [5]VOL_SWEEP [4]FREQ_SWEEP [3]TIMER_SYNC [2]PCM_LOOP [1]FILTER_PHASE_RESET [0]PHASE_RESET
  chWrite(ch, SGU1_CHN_FLAGS1,
    (chan[ch].phaseReset ? SGU1_FLAGS1_PHASE_RESET : 0)
    | (chan[ch].filterPhaseReset ? SGU1_FLAGS1_FILTER_PHASE_RESET : 0)
    | (chan[ch].pcmLoop ? SGU1_FLAGS1_PCM_LOOP : 0)
    | (chan[ch].timerSync ? SGU1_FLAGS1_TIMER_SYNC : 0)
    | (chan[ch].freqSweep ? SGU1_FLAGS1_FREQ_SWEEP : 0)
    | (chan[ch].volSweep ? SGU1_FLAGS1_VOL_SWEEP : 0)
    | (chan[ch].cutSweep ? SGU1_FLAGS1_CUT_SWEEP : 0));
  chan[ch].phaseReset = false;
  chan[ch].filterPhaseReset = false;
}

void DivPlatformSGU::tick(bool sysTick) {
  bool mustHardReset=false;

  for (int i=0; i<SGU_CHNS; i++) {
    chan[i].std.next();

    if (sysTick) {
      if (chan[i].pw_slide!=0) {
        chan[i].virtual_duty-=chan[i].pw_slide;
        chan[i].virtual_duty=CLAMP(chan[i].virtual_duty,0,0xfff);
        chan[i].duty=chan[i].virtual_duty>>5;

        chWrite(i,0x08,chan[i].duty);
      }
      if (chan[i].cutoff_slide!=0) {
        chan[i].cutoff+=chan[i].cutoff_slide*4;
        chan[i].cutoff=CLAMP(chan[i].cutoff,0,0x3fff);

        chWrite(i,0x06,chan[i].cutoff&0xff);
        chWrite(i,0x07,chan[i].cutoff>>8);
      }
    }

    if (chan[i].std.vol.had) {
      DivInstrument* ins=parent->getIns(chan[i].ins,DIV_INS_SU);
      if (ins->type==DIV_INS_AMIGA) {
        chan[i].outVol=((chan[i].vol&127)*MIN(64,chan[i].std.vol.val))>>6;
      } else {
        chan[i].outVol=((chan[i].vol&127)*MIN(127,chan[i].std.vol.val))>>7;
      }
      chWrite(i,0x02,chan[i].outVol);
    }

    if (NEW_ARP_STRAT) {
      chan[i].handleArp();
    } else if (chan[i].std.arp.had) {
      if (!chan[i].inPorta) {
        chan[i].baseFreq=NOTE_FREQUENCY(parent->calcArp(chan[i].note,chan[i].std.arp.val));
      }
      chan[i].freqChanged=true;
    }

    if (chan[i].std.wave.had) {
      chan[i].wave=chan[i].std.wave.val&7;
      writeControl(i);
    }

    if (chan[i].std.panL.had) {
      chan[i].pan=chan[i].std.panL.val;
      chWrite(i,0x03,chan[i].pan);
    }

    if (chan[i].std.pitch.had) {
      if (chan[i].std.pitch.mode) {
        chan[i].pitch2+=chan[i].std.pitch.val;
        CLAMP_VAR(chan[i].pitch2,-32768,32767);
      } else {
        chan[i].pitch2=chan[i].std.pitch.val;
      }
      chan[i].freqChanged=true;
    }

    if (chan[i].std.phaseReset.had) {
      chan[i].phaseReset=chan[i].std.phaseReset.val;
      writeControlUpper(i);
    }

    if (chan[i].std.duty.had) {
      chan[i].duty=chan[i].std.duty.val;
      chan[i].virtual_duty=(unsigned short)chan[i].duty<<5;
      chWrite(i,0x08,chan[i].duty);
    }

    if (chan[i].std.ex1.had) {
      chan[i].cutoff=((chan[i].std.ex1.val&16383)*chan[i].baseCutoff)/16380;
      chWrite(i,0x06,chan[i].cutoff&0xff);
      chWrite(i,0x07,chan[i].cutoff>>8);
    }

    if (chan[i].std.ex2.had) {
      chan[i].res=chan[i].std.ex2.val;
      chWrite(i,0x09,chan[i].res);
    }

    if (chan[i].std.ex3.had) {
      chan[i].control=chan[i].std.ex3.val&15;
      writeControl(i);
    }

    if (chan[i].std.ex4.had) {
      chan[i].syncTimer=chan[i].std.ex4.val&65535;
      chan[i].timerSync=(chan[i].syncTimer>0);
      chWrite(i,SGU1_CHN_RESTIMER_L,chan[i].syncTimer&0xff);
      chWrite(i,SGU1_CHN_RESTIMER_H,chan[i].syncTimer>>8);
      writeControlUpper(i);
    }

    // run hardware sequence (SoundUnit feature)
    if (chan[i].active) {
      if (--chan[i].hwSeqDelay<=0) {
        chan[i].hwSeqDelay=0;
        DivInstrument* ins=parent->getIns(chan[i].ins,DIV_INS_SU);
        int hwSeqCount=0;
        while (chan[i].hwSeqPos<ins->su.hwSeqLen && hwSeqCount<8) {
          bool leave=false;
          unsigned char bound=ins->su.hwSeq[chan[i].hwSeqPos].bound;
          unsigned char val=ins->su.hwSeq[chan[i].hwSeqPos].val;
          unsigned short speed=ins->su.hwSeq[chan[i].hwSeqPos].speed;
          switch (ins->su.hwSeq[chan[i].hwSeqPos].cmd) {
            case DivInstrumentSoundUnit::DIV_SU_HWCMD_VOL:
              chan[i].volSweepP=speed;
              chan[i].volSweepV=val;
              chan[i].volSweepB=bound;
              chan[i].volSweep=(val>0);
              chWrite(i,SGU1_CHN_SWVOL_SPD_L,chan[i].volSweepP&0xff);
              chWrite(i,SGU1_CHN_SWVOL_SPD_H,chan[i].volSweepP>>8);
              chWrite(i,SGU1_CHN_SWVOL_AMT,chan[i].volSweepV);
              chWrite(i,SGU1_CHN_SWVOL_BND,chan[i].volSweepB);
              writeControlUpper(i);
              break;
            case DivInstrumentSoundUnit::DIV_SU_HWCMD_PITCH:
              chan[i].freqSweepP=speed;
              chan[i].freqSweepV=val;
              chan[i].freqSweepB=bound;
              chan[i].freqSweep=(val>0);
              chWrite(i,SGU1_CHN_SWFREQ_SPD_L,chan[i].freqSweepP&0xff);
              chWrite(i,SGU1_CHN_SWFREQ_SPD_H,chan[i].freqSweepP>>8);
              chWrite(i,SGU1_CHN_SWFREQ_AMT,chan[i].freqSweepV);
              chWrite(i,SGU1_CHN_SWFREQ_BND,chan[i].freqSweepB);
              writeControlUpper(i);
              break;
            case DivInstrumentSoundUnit::DIV_SU_HWCMD_CUT:
              chan[i].cutSweepP=speed;
              chan[i].cutSweepV=val;
              chan[i].cutSweepB=bound;
              chan[i].cutSweep=(val>0);
              chWrite(i,SGU1_CHN_SWCUT_SPD_L,chan[i].cutSweepP&0xff);
              chWrite(i,SGU1_CHN_SWCUT_SPD_H,chan[i].cutSweepP>>8);
              chWrite(i,SGU1_CHN_SWCUT_AMT,chan[i].cutSweepV);
              chWrite(i,SGU1_CHN_SWCUT_BND,chan[i].cutSweepB);
              writeControlUpper(i);
              break;
            case DivInstrumentSoundUnit::DIV_SU_HWCMD_WAIT:
              chan[i].hwSeqDelay=(val+1)*parent->tickMult;
              leave=true;
              break;
            case DivInstrumentSoundUnit::DIV_SU_HWCMD_WAIT_REL:
              if (!chan[i].released) {
                chan[i].hwSeqPos--;
                leave=true;
              }
              break;
            case DivInstrumentSoundUnit::DIV_SU_HWCMD_LOOP:
              chan[i].hwSeqPos=val-1;
              break;
            case DivInstrumentSoundUnit::DIV_SU_HWCMD_LOOP_REL:
              if (!chan[i].released) {
                chan[i].hwSeqPos=val-1;
              }
              break;
          }

          chan[i].hwSeqPos++;
          if (leave) break;
          hwSeqCount++;
        }
      }
    }

    for (int o=0; o<SGU_OP_PER_CH; o++) {
      DivInstrumentFM::Operator& op=chan[i].state.fm.op[o];
      DivInstrumentESFM::Operator& opE=chan[i].state.esfm.op[o];
      DivMacroInt::IntOp& m=chan[i].std.op[o];
      bool opDirty=false;

      if (m.am.had) {
        op.am=m.am.val;
        opDirty=true;
      }
      if (m.vib.had) {
        op.vib=m.vib.val;
        opDirty=true;
      }
      if (m.sus.had) {
        op.sus=m.sus.val;
        opDirty=true;
      }
      if (m.ksr.had) {
        op.ksr=m.ksr.val;
        opDirty=true;
      }
      if (m.mult.had) {
        op.mult=m.mult.val;
        opDirty=true;
      }

      if (m.ar.had) {
        op.ar=m.ar.val;
        opDirty=true;
      }
      if (m.dr.had) {
        op.dr=m.dr.val;
        opDirty=true;
      }
      if (m.sl.had) {
        op.sl=m.sl.val;
        opDirty=true;
      }
      if (m.rr.had) {
        op.rr=m.rr.val;
        opDirty=true;
      }

      if (m.tl.had || m.ksl.had) {
        if (m.tl.had) {
          op.tl=m.tl.val&63;
        }
        if (m.ksl.had) {
          op.ksl=m.ksl.val;
        }
        opWrite(i,o,SGU_OP_REG_TL,(op.tl&SGU_OP1_TL_MASK)|(op.ksl<<SGU_OP1_KSL_SHIFT));
      }

      if (m.dam.had) {
        op.dam=m.dam.val;
        opDirty=true;
      }
      if (m.dvb.had) {
        op.dvb=m.dvb.val;
        opDirty=true;
      }
      if (m.rs.had) {
        // operator panning
        opE.left=(m.rs.val&2)!=0;
        opE.right=(m.rs.val&1)!=0;
        opDirty=true;
      }
      if (m.d2r.had) {
        // modIn
        opE.modIn=m.d2r.val;
        opDirty=true;
      }

      if (m.egt.had) {
        // outLvl
        opE.outLvl=m.egt.val;
        opDirty=true;
      }
      if (m.ws.had) {
        op.ws=m.ws.val;
        opDirty=true;
      }

      // detune/fixed pitch
      if (opE.fixed) {
        if (m.ssg.had) {
          opE.ct=(opE.ct&(~(7<<2)))|((m.ssg.val&7)<<2);
          chan[i].freqChanged=true;
        }
        if (m.dt.had) {
          opE.dt=m.dt.val&0xff;
          opE.ct=(opE.ct&(~3))|((m.dt.val>>8)&3);
          chan[i].freqChanged=true;
        }
      } else {
        chan[i].handleArpFmOp(0, o);
        chan[i].handlePitchFmOp(o);
      }

      if (m.dt2.had) {
        opE.delay=m.dt2.val;
        opDirty=true;
      }

      if (opDirty) {
        DivInstrumentFM::Operator& op=chan[i].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[i].state.esfm.op[o];
        applyOpRegs(i,o,op,opE);
      }
    }

    if (chan[i].hardReset && chan[i].keyOn) {
      mustHardReset=true;
      for (int o=0; o<SGU_OP_PER_CH; o++) {
        opWrite(i,o,SGU_OP_REG_SL_RR,0xff);  // SL=15, RR=15: immediate silence
      }
    }

    if (chan[i].freqChanged || chan[i].keyOn || chan[i].keyOff) {
      // SGU uses direct 16-bit frequency per channel (operators derive from MUL/DT)
      chan[i].freq=parent->calcFreq(chan[i].baseFreq,chan[i].pitch,chan[i].fixedArp?chan[i].baseNoteOverride:chan[i].arpOff,chan[i].fixedArp,false,2,chan[i].pitch2,chipClock,CHIP_FREQBASE);
      // Apply sample rate adjustment for PCM playback
      if (chan[i].pcm) {
        DivSample* sample=parent->getSample(chan[i].sample);
        if (sample!=NULL) {
          double off=0.25;
          if (sample->centerRate<1) {
            off=0.25;
          } else {
            off=(double)sample->centerRate/(parent->getCenterRate()*4.0);
          }
          chan[i].freq=(double)chan[i].freq*off;
        }
      }
      if (chan[i].freq<0) chan[i].freq=0;
      if (chan[i].freq>65535) chan[i].freq=65535;

      // Write channel frequency
      chWrite(i,SGU1_CHN_FREQ_L,chan[i].freq&0xff);
      chWrite(i,SGU1_CHN_FREQ_H,(chan[i].freq>>8)&0xff);

      // PCM sample setup on keyOn
      if (chan[i].keyOn) {
        if (chan[i].pcm) {
          int sNum=chan[i].sample;
          DivSample* sample=parent->getSample(sNum);
          if (sample!=NULL && sNum>=0 && sNum<parent->song.sampleLen) {
            unsigned int sampleEnd=sampleOffSGU[sNum]+(sample->getLoopEndPosition());
            unsigned int off=sampleOffSGU[sNum]+chan[i].hasOffset;
            chan[i].hasOffset=0;
            if (sampleEnd>=getSampleMemCapacity(0)) sampleEnd=getSampleMemCapacity(0)-1;
            chWrite(i,SGU1_CHN_PCM_POS_L,off&0xff);
            chWrite(i,SGU1_CHN_PCM_POS_H,off>>8);
            chWrite(i,SGU1_CHN_PCM_END_L,sampleEnd&0xff);
            chWrite(i,SGU1_CHN_PCM_END_H,sampleEnd>>8);
            if (sample->isLoopable()) {
              unsigned int sampleLoop=sampleOffSGU[sNum]+sample->getLoopStartPosition();
              if (sampleLoop>=getSampleMemCapacity(0)) sampleLoop=getSampleMemCapacity(0)-1;
              chWrite(i,SGU1_CHN_PCM_RST_L,sampleLoop&0xff);
              chWrite(i,SGU1_CHN_PCM_RST_H,sampleLoop>>8);
              chan[i].pcmLoop=true;
            } else {
              chan[i].pcmLoop=false;
            }
            writeControl(i);
            writeControlUpper(i);
          }
        }
      }

      if (chan[i].keyOn && !chan[i].hardReset) {
        // Key on: First clear GATE to ensure 0→1 edge transition for envelope retrigger
        chan[i].gate=false;
        writeControl(i);
        chan[i].gate=true;
        writeControl(i);
      }
      if (chan[i].keyOn) chan[i].keyOn=false;
      if (chan[i].keyOff) chan[i].keyOff=false;
      chan[i].freqChanged=false;
    }
  }

  if (mustHardReset) {
    for (int i=0; i<SGU_CHNS; i++) {
      if (chan[i].hardReset && chan[i].keyOn) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[i].state.fm.op[o];
          opWrite(i,o,SGU_OP_REG_SL_RR,(op.sl<<SGU_OP3_SL_SHIFT)|(op.rr&SGU_OP3_RR_MASK));
        }
        // Key on after hard reset: First clear GATE to ensure 0→1 edge transition
        chan[i].gate=false;
        writeControl(i);
        chan[i].gate=true;
        writeControl(i);
        chan[i].keyOn=false;
      }
    }
  }
}

void DivPlatformSGU::muteChannel(int ch, bool mute) {
  isMuted[ch]=mute;
  chip.muted[ch]=mute;
}

void DivPlatformSGU::commitState(int ch, DivInstrument* ins) {
  if (ins==NULL) return;

  if (ins->type==DIV_INS_AMIGA || ins->amiga.useSample) {
    chan[ch].pcm=true;
    writeControl(ch);
    writeControlUpper(ch);
    return;
  }

  chan[ch].pcm=false;

  DivInstrumentFM fm=ins->fm;
  DivInstrumentESFM esfm=ins->esfm;

  for (int o=0; o<SGU_OP_PER_CH; o++) {

  static const unsigned char oplToSguWaveformMap[8]={
    /* 0: SINE         -> */ SGU_WAVE_SINE,
    /* 1: HALF_SINE    -> */ SGU_WAVE_PULSE,
    /* 2: ABS_SINE     -> */ SGU_WAVE_SINE,
    /* 3: PULSE_SINE   -> */ SGU_WAVE_TRIANGLE,
    /* 4: ALT_SINE     -> */ SGU_WAVE_PULSE,
    /* 5: ABS_ALT_SINE -> */ SGU_WAVE_PULSE,
    /* 6: SQUARE       -> */ SGU_WAVE_PULSE,
    /* 7: LOG_SAW      -> */ SGU_WAVE_SAWTOOTH,
  };

  /* Convert waveforms */
  switch (ins->type) {
    case DIV_INS_OPM:
      // OPM has no waveform select (sine only)
      fm.op[o].ws=SGU_WAVE_SINE;
      break;
    case DIV_INS_ESFM:
    case DIV_INS_FM:
    case DIV_INS_OPL:
    case DIV_INS_OPLL:
    case DIV_INS_OPZ:
      fm.op[o].ws=oplToSguWaveformMap[fm.op[o].ws & 0x07];
      break;
    default:
      break;
  }

  /* ### FM operator parameter support matrix
         When shown, it's bits per parameter

          | OPL | OPM | OPN | OPN2| OPZ | OPLL| ESFM| SGU |
    -------------------------------------------------------
      AR  |  4  |  5  |  5  |  5  |  5  |  4  |  4  |  5  |
 D1R  DR  |  4  |  5  |  5  |  5  |  5  |  4  |  4  |  5  |
 D1L  SL  |  4  |  4  |  4  |  4  |  4  |  4  |  4  |  4  |
      SR  |  1  |  5  |  5  |  5  |  5  |  1  |  1  |  5  |
 D2R  RR  |  4  |  4  |  4  |  4  |  4  |  4  |  4  |  4  |
      TL  |  6  |  7  |  7  |  7  |  7  | 6/4 |  6  |  6  |
      KSL |  2  |  -  |  -  |  -  |  -  |  2  |  2  |  2  |
      KSR |  1  |  2  |  2  |  2  |  2  |  1  |  1  |  2  |
    SSG-EG|  -  |  -  |  -  |  4  |  -  |  -  |  -  |  3  |
      DT  |  -  |  3  |  3  |  3  |  3  |  -  |  -  |  -  |
      DT2 |  -  |  2  |  -  |  -  |  -  |  -  |  -  |  -  |
  AM  TRM |  1  |  1  |  -  |  1  |  1  |  1  |  1  |  1  |
      TRMD|  1  |  2  |  -  |  2  |  2  |  -  |  1  |  1  |
  FM  VIB |  1  |  -  |  -  |  -  |  -  |  1  |  1  |  1  |
      VIBD|  1  |  3  |  -  |  3  |  3  |  -  |  1  |  1  |
      MULT|  4  |  4  |  4  |  4  |     |  4  |  4  |  4  |
      FIX |  -  |  -  |  -  |  -  |  1  |  -  |  -  |  1  |
      MOD |  -  |  -  |  -  |     |  -  | *1) |  3  |  3  |
      OUT |  -  |  -  |  -  |     |  -  |  4  |  3  |  3  |
      WAV | 2/3 |  -  |  -  |  -  |  3  |  1  |  3  |  3  |
      DEL |  -  |  -  |  -  |  -  |  -  |  -  |  3  |  3  |
      WPAR|  -  |  -  |  -  |  -  |  -  |  -  |  -  |  3  |
      SYNC|  -  |  -  |  -  |  -  |  -  |  -  |  -  |  1  |
      RING|  -  |  -  |  -  |  -  |  -  |  -  |  -  |  1  |

      *1) - OPLL Reg 03
        DC - Reg 03, Bit 4: carrier waveform select;   1 = “distortion” waveform = flat/rectified (“half-sine”) variant
        DM - Reg 03, Bit 3: modulator waveform select; 1 = flat/rectified (“half-sine”) variant
        FB - Reg 03, Bits 2-0: feedback amount for the modulator (operator 1), 0..7
  */

  /* OPN2 - difficult to map on linear ESFM with outs…
    Algorithm #	Layout	   Suggested uses
            0   1-2-3-4->  Distortion guitar, "high hat chopper" (?) bass
            1   1-+3-4->   Harp, PSG (programmable sound generator) sound
                2/
            2   1---+4->   Bass, electric guitar, brass, piano, woods
                2-3/
            3   1-2-+4->   Strings, folk guitar, chimes
                  3/
            4   1-2-+->    Flute, bells, chorus, bass drum, snare drum, tom-tom
                3-4/
            5      /-2-\   Brass, organ
                1-+--3--+->
                   \-4-/
            6   1-2-\      Xylophone, tom-tom, organ, vibraphone, snare drum, base drum
                  3--+->
                  4-/
            7   1-\        Pipe organ
                2-+->
                3-/
                4-/

  */

  /* Convert ADSR envelope parameters */
  switch (ins->type) {
    case DIV_INS_OPLL:
      {
        // Convert KSR and apply global AMS/FMS
        fm.op[o].rs=(fm.op[o].ksr&1)?3:0;
        fm.op[o].dam=fm.ams&1;
        fm.op[o].dvb=fm.fms&1;
      }
      [[fallthrough]];
      case DIV_INS_OPL:
      case DIV_INS_ESFM:
      {
        // AR/DR need shifting: ESFM/OPL uses 4-bit rates, SGU expects 5-bit
        fm.op[o].ar=(unsigned char)(((fm.op[o].ar & 0x0f) << 1) | 1);
        fm.op[o].dr=(unsigned char)(((fm.op[o].dr & 0x0f) << 1) | 1);
        // EGT=0: infinite sustain (SR=0), EGT=1: no sustain (SR=max, skip to release)
        fm.op[o].d2r = fm.op[o].egt ? 0x1f : 0;
      }
      break;
    case DIV_INS_FM:
    case DIV_INS_OPM:
    case DIV_INS_OPZ:
      // OPN/OPM/OPZ already have 5-bit AR/DR and 7-bit TL - use as-is
      break;
    case DIV_INS_C64:
      {
        // C64 uses single carrier on op3; disable modulators
        if (o==3) {
          // Convert C64 ADSR (4-bit each) to FM envelope on carrier
          const unsigned char decay=(ins->c64.s==15)?0:(ins->c64.d&0x0f);
          fm.op[o].ar=(unsigned char)(((ins->c64.a & 0x0f) << 1) | 1);
          fm.op[o].dr=(unsigned char)(((decay & 0x0f) << 1) | 1);
          fm.op[o].sl=(unsigned char)(15-(ins->c64.s&0x0f));
          fm.op[o].rr=ins->c64.r&0x0f;
          fm.op[o].d2r=0;
          fm.op[o].tl=0;
          fm.op[o].mult=1;
          fm.op[o].ws=sguC64Wave(ins->c64,false);
        } else {
          // Disable modulator operators
          fm.op[o].tl=127;
          fm.op[o].enable=false;
        }
      }
      break;
    case DIV_INS_SID2:
      {
        // SID2 uses single carrier on op3; disable modulators
        if (o==3) {
          const bool periodicNoise=(ins->sid2.noiseMode!=0);
          const unsigned char decay=(ins->c64.s==15)?0:(ins->c64.d&0x0f);
          fm.op[o].ar=(unsigned char)(((ins->c64.a & 0x0f) << 1) | 1);
          fm.op[o].dr=(unsigned char)(((decay & 0x0f) << 1) | 1);
          fm.op[o].sl=(unsigned char)(15-(ins->c64.s&0x0f));
          fm.op[o].rr=ins->c64.r&0x0f;
          fm.op[o].d2r=0;
          fm.op[o].tl=0;
          fm.op[o].mult=1;
          fm.op[o].ws=sguC64Wave(ins->c64,periodicNoise);
        } else {
          fm.op[o].tl=127;
          fm.op[o].enable=false;
        }
      }
      break;
    case DIV_INS_SU:
      {
        // SoundUnit uses single carrier on op3 with simple envelope
        if (o==3) {
          fm.op[o].ar=31;
          fm.op[o].dr=0;
          fm.op[o].sl=0;
          fm.op[o].rr=15;
          fm.op[o].d2r=0;
          fm.op[o].tl=0;
          fm.op[o].mult=1;
          fm.op[o].ws=SGU_WAVE_SAWTOOTH;
        } else {
          fm.op[o].tl=127;
          fm.op[o].enable=false;
        }
      }
      break;
    case DIV_INS_POKEY:
      {
        // POKEY uses single carrier on op3 with instant envelope
        if (o==3) {
          fm.op[o].ar=31;
          fm.op[o].dr=0;
          fm.op[o].sl=0;
          fm.op[o].rr=15;
          fm.op[o].d2r=0;
          fm.op[o].tl=0;
          fm.op[o].mult=1;
          fm.op[o].ws=SGU_WAVE_PULSE;
        } else {
          fm.op[o].tl=127;
          fm.op[o].enable=false;
        }
      }
      break;
    default:
      break;
  }

  /* Convert Misc parameters */
  switch (ins->type) {
    case DIV_INS_OPZ:
      {
        // OPZ uses EGT flag for fixed frequency mode
        esfm.op[o].fixed=fm.op[o].egt;
      }
      [[fallthrough]];
    case DIV_INS_FM:
    case DIV_INS_OPM:
      {
        // Map UI detune to Yamaha DT1 encoding for Yamaha FM chips (matches platform replayers).
        static const unsigned char dtTable[8]={7,6,5,0,1,2,3,4};
        fm.op[o].dt=dtTable[fm.op[o].dt&7];
      }
      break;
    default:
      break;
  }
  }

  /* Convert operator algorithms */
  switch (ins->type) {
    case DIV_INS_ESFM:
      // ESFM is native format for SGU - use as-is
      break;
    case DIV_INS_AMIGA:
      // Handled by PCM path above
      break;
    case DIV_INS_FM:  // OPN-style (YM2612, YM2203) - registers ordered Op1,Op3,Op2,Op4
    case DIV_INS_OPM: // OPM-style (YM2151) - registers ordered Op1,Op2,Op3,Op4
    case DIV_INS_OPZ: // OPZ-style (TX81Z) - like OPM with fixed frequency support
      // All share the same 8 algorithm topologies inherited from DX9.
      // Furnace normalizes operator ordering internally, so routing is identical.
      // AR/DR are 5-bit, TL is 7-bit; scaling already done in first switch
      // esfm initialized before operator loop; OPZ fixed flags set in misc params switch
      {
        const unsigned char fb=fm.fb&7;
        // Algorithm mapping to ESFM operator routing:
        switch (fm.alg & 7) {
          case 0: // 1→2→3→4→out (serial)
            setEsfmRouting(esfm,
              (const unsigned char[]){fb,7,7,7},
              (const unsigned char[]){0,0,0,7});
            break;
          case 1: // (1+2)→3→4→out
            setEsfmRouting(esfm,
              (const unsigned char[]){fb,0,7,7},
              (const unsigned char[]){0,0,0,7});
            break;
          case 2: // 1+(2→3)→4→out
            setEsfmRouting(esfm,
              (const unsigned char[]){fb,0,7,7},
              (const unsigned char[]){0,0,0,7});
            break;
          case 3: // (1→2)+3→4→out
            setEsfmRouting(esfm,
              (const unsigned char[]){fb,7,0,7},
              (const unsigned char[]){0,0,0,7});
            break;
          case 4: // (1→2)+(3→4)→out
            setEsfmRouting(esfm,
              (const unsigned char[]){fb,7,0,7},
              (const unsigned char[]){0,7,0,7});
            break;
          case 5: // 1→(2+3+4)→out
            setEsfmRouting(esfm,
              (const unsigned char[]){fb,7,7,7},
              (const unsigned char[]){0,7,7,7});
            break;
          case 6: // (1→2)+3+4→out
            setEsfmRouting(esfm,
              (const unsigned char[]){fb,7,0,0},
              (const unsigned char[]){0,7,7,7});
            break;
          case 7: // 1+2+3+4→out (additive)
            setEsfmRouting(esfm,
              (const unsigned char[]){fb,0,0,0},
              (const unsigned char[]){7,7,7,7});
            break;
        }
      }
      break;
    case DIV_INS_OPL:
      // OPL-style 2-operator FM
      // Only operators 0 and 1 are used; disable 2 and 3
      {
        esfm=DivInstrumentESFM();
        // OPL algorithm: 0 = modulator→carrier, 1 = additive
        const bool algAdd=(fm.alg&1);
        esfm.op[0].modIn=fm.fb&7;
        esfm.op[0].outLvl=algAdd?7:0;
        esfm.op[1].modIn=algAdd?0:7;
        esfm.op[1].outLvl=7;
        // Disable operators 2 and 3
        for (int i=2; i<4; i++) {
          fm.op[i].enable=false;
          fm.op[i].tl=127;
          esfm.op[i].outLvl=0;
          esfm.op[i].modIn=0;
        }
      }
      break;
    case DIV_INS_OPLL:
      // OPLL-style 2-operator FM (very similar to OPL)
      {
        esfm=DivInstrumentESFM();
        // Scale TL for carrier (op1) from 4-bit to 7-bit
        fm.op[1].tl=(unsigned char)(((fm.op[1].tl & 0x0f) << 3) | 0x04);
        // Convert KSR
        for (int i=0; i<2; i++) {
          fm.op[i].rs=(fm.op[i].ksr&1)?3:0;
          fm.op[i].dam=fm.ams&1;
          fm.op[i].dvb=fm.fms&1;
        }
        const bool algAdd=(fm.alg&1);
        esfm.op[0].modIn=fm.fb&7;
        esfm.op[0].outLvl=algAdd?7:0;
        esfm.op[1].modIn=algAdd?0:7;
        esfm.op[1].outLvl=7;
        // Disable operators 2 and 3
        for (int i=2; i<4; i++) {
          fm.op[i].enable=false;
          fm.op[i].tl=127;
          esfm.op[i].outLvl=0;
          esfm.op[i].modIn=0;
        }
      }
      break;
    case DIV_INS_SU:
    case DIV_INS_POKEY:
      // Single oscillator mapped to op3 - set routing only
      // ADSR conversion handled in first switch
      {
        esfm=DivInstrumentESFM();
        for (int o=0; o<3; o++) {
          esfm.op[o].outLvl=0;
          esfm.op[o].modIn=0;
        }
        esfm.op[3].modIn=0;
        esfm.op[3].outLvl=7;
      }
      break;
    case DIV_INS_C64:
    case DIV_INS_SID2:
      // C64/SID2 - single carrier on op3, plus channel settings
      // ADSR conversion handled in first switch
      {
        esfm=DivInstrumentESFM();
        for (int o=0; o<3; o++) {
          esfm.op[o].outLvl=0;
          esfm.op[o].modIn=0;
        }
        esfm.op[3].modIn=0;
        esfm.op[3].outLvl=7;

        // Handle duty cycle
        if (ins->c64.resetDuty || chan[ch].insChanged) {
          const unsigned short dutyClamp=(ins->c64.duty>4095)?4095:ins->c64.duty;
          chan[ch].duty=(unsigned char)(dutyClamp>>5);
          chan[ch].virtual_duty=(unsigned short)chan[ch].duty<<5;
          chWrite(ch,SGU1_CHN_DUTY,chan[ch].duty);
        }

        // Handle filter settings
        bool updateFilter=false;
        if (!ins->c64.toFilter) {
          if (chan[ch].control!=0) {
            chan[ch].control=0;
            updateFilter=true;
          }
        } else if (ins->c64.initFilter || chan[ch].insChanged) {
          if (ins->c64.initFilter) {
            const unsigned int cutClamp=(ins->c64.cut>2047)?2047:ins->c64.cut;
            chan[ch].cutoff=(unsigned short)((cutClamp*65535U+1023U)/2047U);
            chan[ch].baseCutoff=chan[ch].cutoff;
            const unsigned char resClamp=ins->c64.res&0x0f;
            chan[ch].res=(unsigned char)((resClamp<<4)|resClamp);
            chan[ch].control=(ins->c64.lp?2:0)|(ins->c64.hp?4:0)|(ins->c64.bp?8:0);
            updateFilter=true;
          }
        }
        if (updateFilter) {
          chWrite(ch,SGU1_CHN_CUTOFF_L,chan[ch].cutoff&0xff);
          chWrite(ch,SGU1_CHN_CUTOFF_H,chan[ch].cutoff>>8);
          chWrite(ch,SGU1_CHN_RESON,chan[ch].res);
          writeControl(ch);
        }
      }
      break;
    default:
      break;
  }

  chan[ch].state.fm=fm;
  chan[ch].state.esfm=esfm;

  for (int o=0; o<SGU_OP_PER_CH; o++) {
    applyOpRegs(ch, o, fm.op[o], esfm.op[o]);
  }
}

int DivPlatformSGU::dispatch(DivCommand c) {
  switch (c.cmd) {
    case DIV_CMD_NOTE_ON: {
      DivInstrument* ins=parent->getIns(chan[c.chan].ins,DIV_INS_ESFM);

      chan[c.chan].macroInit(ins);
      memset(chan[c.chan].opsState, 0, sizeof(chan[c.chan].opsState));
      if (!chan[c.chan].std.vol.will) {
        chan[c.chan].outVol=chan[c.chan].vol;
      }

      // Handle PCM mode transition
      if (chan[c.chan].pcm && !(ins->type==DIV_INS_AMIGA || ins->amiga.useSample)) {
        chan[c.chan].pcm=(ins->type==DIV_INS_AMIGA || ins->amiga.useSample);
        writeControl(c.chan);
        writeControlUpper(c.chan);
      }
      chan[c.chan].pcm=(ins->type==DIV_INS_AMIGA || ins->amiga.useSample);

      // Handle PCM sample lookup
      if (chan[c.chan].pcm) {
        if (c.value!=DIV_NOTE_NULL) {
          chan[c.chan].sample=ins->amiga.getSample(c.value);
          chan[c.chan].sampleNote=c.value;
          c.value=ins->amiga.getFreq(c.value);
          chan[c.chan].sampleNoteDelta=c.value-chan[c.chan].sampleNote;
        }
      } else {
        chan[c.chan].sampleNote=DIV_NOTE_NULL;
        chan[c.chan].sampleNoteDelta=0;
      }

      commitState(c.chan,ins);
      chan[c.chan].insChanged=false;
      chWrite(c.chan,SGU1_CHN_VOL,chan[c.chan].outVol);

      if (c.value!=DIV_NOTE_NULL) {
        chan[c.chan].baseFreq=NOTE_FREQUENCY(c.value);
        chan[c.chan].note=c.value;
        chan[c.chan].freqChanged=true;
      }
      chan[c.chan].keyOn=true;
      chan[c.chan].active=true;
      chan[c.chan].released=false;
      chan[c.chan].hwSeqPos=0;
      chan[c.chan].hwSeqDelay=0;
      break;
    }
    case DIV_CMD_NOTE_OFF:
      chan[c.chan].active=false;
      chan[c.chan].keyOff=true;
      chan[c.chan].keyOn=false;
      chan[c.chan].hwSeqPos=0;
      chan[c.chan].hwSeqDelay=0;
      chan[c.chan].macroInit(NULL);
      break;
    case DIV_CMD_NOTE_OFF_ENV:
      chan[c.chan].active=false;
      chan[c.chan].keyOff=true;
      chan[c.chan].keyOn=false;
      chan[c.chan].std.release();
      break;
    case DIV_CMD_ENV_RELEASE:
      chan[c.chan].std.release();
      chan[c.chan].released=true;
      break;
    case DIV_CMD_VOLUME: {
      chan[c.chan].vol=c.value;
      if (!chan[c.chan].std.vol.has) {
        chan[c.chan].outVol=c.value;
      }
      chWrite(c.chan,SGU1_CHN_VOL,chan[c.chan].outVol);
      break;
    }
    case DIV_CMD_GET_VOLUME:
      return chan[c.chan].vol;
      break;
    case DIV_CMD_INSTRUMENT:
      if (chan[c.chan].ins!=c.value || c.value2==1) {
        chan[c.chan].insChanged=true;
      }
      chan[c.chan].ins=c.value;
      break;
    case DIV_CMD_PANNING: {
      // SGU panning is per-channel, int8_t range (-128 to +127)
      // c.value-c.value2 gives -255 to +255, so scale by 2
      chan[c.chan].pan=(signed char)((c.value-c.value2)/2);
      chWrite(c.chan,SGU1_CHN_PAN,chan[c.chan].pan);
      break;
    }
    case DIV_CMD_PITCH:
      chan[c.chan].pitch=c.value;
      chan[c.chan].freqChanged=true;
      break;
    case DIV_CMD_NOTE_PORTA: {
      int destFreq=NOTE_FREQUENCY(c.value2+chan[c.chan].sampleNoteDelta);
      bool return2=false;
      if (destFreq>chan[c.chan].baseFreq) {
        chan[c.chan].baseFreq+=c.value*((parent->song.compatFlags.linearPitch)?1:(1+(chan[c.chan].baseFreq>>9)));
        if (chan[c.chan].baseFreq>=destFreq) {
          chan[c.chan].baseFreq=destFreq;
          return2=true;
        }
      } else {
        chan[c.chan].baseFreq-=c.value*((parent->song.compatFlags.linearPitch)?1:(1+(chan[c.chan].baseFreq>>9)));
        if (chan[c.chan].baseFreq<=destFreq) {
          chan[c.chan].baseFreq=destFreq;
          return2=true;
        }
      }
      chan[c.chan].freqChanged=true;
      if (return2) {
        chan[c.chan].inPorta=false;
        return 2;
      }
      break;
    }
    case DIV_CMD_LEGATO: {
      if (chan[c.chan].insChanged) {
        DivInstrument* ins=parent->getIns(chan[c.chan].ins,DIV_INS_ESFM);
        commitState(c.chan,ins);
        chan[c.chan].insChanged=false;
      }
      chan[c.chan].baseFreq=NOTE_FREQUENCY(c.value+chan[c.chan].sampleNoteDelta+((HACKY_LEGATO_MESS)?(chan[c.chan].std.arp.val):(0)));
      chan[c.chan].note=c.value;
      chan[c.chan].freqChanged=true;
      break;
    }
    case DIV_CMD_FM_MULT: {
      unsigned int o=c.value;
      if (o >= 4) break;
      DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
      DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
      op.mult=c.value2&15;
      applyOpRegs(c.chan,o,op,opE);
      break;
    }
    case DIV_CMD_FM_TL: {
      unsigned int o=c.value;
      if (o >= 4) break;
      DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
      DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
      op.tl=c.value2&63;
      applyOpRegs(c.chan,o,op,opE);
      break;
    }
    case DIV_CMD_FM_AR: {
      if (c.value<0) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
          DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
          op.ar=c.value2&31;
          applyOpRegs(c.chan,o,op,opE);
        }
      } else {
        unsigned int o=c.value;
        if (o >= 4) break;
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        op.ar=c.value2&31;
        applyOpRegs(c.chan,o,op,opE);
      }
      break;
    }
    case DIV_CMD_FM_DR: {
      if (c.value<0) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
          DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
          op.dr=c.value2&31;
          applyOpRegs(c.chan,o,op,opE);
        }
      } else {
        unsigned int o=c.value;
        if (o >= 4) break;
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        op.dr=c.value2&31;
        applyOpRegs(c.chan,o,op,opE);
      }
      break;
    }
    case DIV_CMD_FM_SL: {
      if (c.value<0) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
          DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
          op.sl=c.value2&15;
          applyOpRegs(c.chan,o,op,opE);
        }
      } else {
        unsigned int o=c.value;
        if (o >= 4) break;
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        op.sl=c.value2&15;
        applyOpRegs(c.chan,o,op,opE);
      }
      break;
    }
    case DIV_CMD_FM_RR: {
      if (c.value<0) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
          DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
          op.rr=c.value2&15;
          applyOpRegs(c.chan,o,op,opE);
        }
      } else {
        unsigned int o=c.value;
        if (o >= 4) break;
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        op.rr=c.value2&15;
        applyOpRegs(c.chan,o,op,opE);
      }
      break;
    }
    case DIV_CMD_FM_AM: {
      if (c.value<0) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
          DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
          op.am=c.value2&1;
          applyOpRegs(c.chan,o,op,opE);
        }
      } else {
        unsigned int o=c.value;
        if (o >= 4) break;
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        op.am=c.value2&1;
        applyOpRegs(c.chan,o,op,opE);
      }
      break;
    }
    case DIV_CMD_FM_VIB: {
      if (c.value<0) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
          DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
          op.vib=c.value2&1;
          applyOpRegs(c.chan,o,op,opE);
        }
      } else {
        unsigned int o=c.value;
        if (o >= 4) break;
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        op.vib=c.value2&1;
        applyOpRegs(c.chan,o,op,opE);
      }
      break;
    }
    case DIV_CMD_FM_SUS: {
      if (c.value<0) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
          DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
          op.sus=c.value2&1;
          applyOpRegs(c.chan,o,op,opE);
        }
      } else {
        unsigned int o=c.value;
        if (o >= 4) break;
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        op.sus=c.value2&1;
        applyOpRegs(c.chan,o,op,opE);
      }
      break;
    }
    case DIV_CMD_FM_KSR: {
      if (c.value<0) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
          DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
          op.ksr=c.value2&1;
          applyOpRegs(c.chan,o,op,opE);
        }
      } else {
        unsigned int o=c.value;
        if (o >= 4) break;
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        op.ksr=c.value2&1;
        applyOpRegs(c.chan,o,op,opE);
      }
      break;
    }
    case DIV_CMD_FM_WS: {
      if (c.value<0) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
          DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
          op.ws=c.value2&7;
          applyOpRegs(c.chan,o,op,opE);
        }
      } else {
        unsigned int o=c.value;
        if (o >= 4) break;
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        op.ws=c.value2&7;
        applyOpRegs(c.chan,o,op,opE);
      }
      break;
    }
    // KSL
    case DIV_CMD_FM_RS: {
      if (c.value<0) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
          DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
          op.ksl=c.value2&3;
          applyOpRegs(c.chan,o,op,opE);
        }
      } else {
        unsigned int o=c.value;
        if (o >= 4) break;
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        op.ksl=c.value2&3;
        applyOpRegs(c.chan,o,op,opE);
      }
      break;
    }
    case DIV_CMD_FM_AM_DEPTH: {
      unsigned int o=c.value;
      if (o >= 4) break;
      DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
      DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
      op.dam=c.value2&1;
      applyOpRegs(c.chan,o,op,opE);
      break;
    }
    case DIV_CMD_FM_PM_DEPTH: {
      unsigned int o=c.value;
      if (o >= 4) break;
      DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
      DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
      op.dvb=c.value2&1;
      applyOpRegs(c.chan,o,op,opE);
      break;
    }
    case DIV_CMD_FM_FIXFREQ: {
      unsigned int o=c.value&3;
      bool isFNum=c.value&4;
      DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
      if (!opE.fixed) break;
      if (isFNum) {
        opE.dt=(c.value2)&0xff;
        opE.ct=(opE.ct&(~3))|((c.value2>>8)&3);
        chan[c.chan].freqChanged=true;
      } else {
        opE.ct=(opE.ct&(~(7<<2)))|((c.value2&7)<<2);
        chan[c.chan].freqChanged=true;
      }
      break;
    }
    case DIV_CMD_ESFM_OP_PANNING: {
      unsigned int o=c.value;
      if (o >= 4) break;
      DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
      DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
      opE.left=(c.value2&0xf0)!=0;
      opE.right=(c.value2&0x0f)!=0;
      applyOpRegs(c.chan,o,op,opE);
      break;
    }
    case DIV_CMD_ESFM_OUTLVL: {
      if (c.value<0) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
          DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
          opE.outLvl=c.value2&7;
          applyOpRegs(c.chan,o,op,opE);
        }
      } else {
        unsigned int o=c.value;
        if (o >= 4) break;
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        opE.outLvl=c.value2&7;
        applyOpRegs(c.chan,o,op,opE);
      }
      break;
    }
    case DIV_CMD_ESFM_MODIN: {
      if (c.value<0) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
          DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
          opE.modIn=c.value2&7;
          applyOpRegs(c.chan,o,op,opE);
        }
      } else {
        unsigned int o=c.value;
        if (o >= 4) break;
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        opE.modIn=c.value2&7;
        applyOpRegs(c.chan,o,op,opE);
      }
      break;
    }
    case DIV_CMD_ESFM_ENV_DELAY: {
      if (c.value<0) {
        for (int o=0; o<SGU_OP_PER_CH; o++) {
          DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
          DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
          opE.delay=c.value2&7;
          applyOpRegs(c.chan,o,op,opE);
        }
      } else {
        unsigned int o=c.value;
        if (o >= 4) break;
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        opE.delay=c.value2&7;
        applyOpRegs(c.chan,o,op,opE);
      }
      break;
    }
    case DIV_CMD_STD_NOISE_MODE: {
      DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[3];
      DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[3];
      chan[c.chan].state.esfm.noise=c.value&3;
      applyOpRegs(c.chan,3,op,opE);
      break;
    }
    case DIV_CMD_FM_DT: {
      unsigned int o=c.value;
      if (o >= 4) break;
      DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
      if (opE.fixed) break;
      opE.dt=c.value2-0x80;
      chan[c.chan].freqChanged=true;
      break;
    }
    case DIV_CMD_FM_HARD_RESET:
      chan[c.chan].hardReset=c.value;
      break;
    case DIV_CMD_MACRO_OFF:
      chan[c.chan].std.mask(c.value,true);
      break;
    case DIV_CMD_MACRO_ON:
      chan[c.chan].std.mask(c.value,false);
      break;
    case DIV_CMD_MACRO_RESTART:
      chan[c.chan].std.restart(c.value);
      break;
    case DIV_CMD_GET_VOLMAX:
      return 63;
      break;
    case DIV_CMD_PRE_PORTA:
      if (chan[c.chan].active && c.value2) {
        if (parent->song.compatFlags.resetMacroOnPorta) chan[c.chan].macroInit(parent->getIns(chan[c.chan].ins,DIV_INS_ESFM));
      }
      if (!chan[c.chan].inPorta && c.value && !parent->song.compatFlags.brokenPortaArp && chan[c.chan].std.arp.will && !NEW_ARP_STRAT) {
        chan[c.chan].baseFreq=NOTE_FREQUENCY(chan[c.chan].note);
      }
      chan[c.chan].inPorta=c.value;
      break;
    case DIV_CMD_C64_EXTENDED:
      switch (c.value>>4) {
        case 0x0: // attack - set AR on all operators (convert 4-bit C64 to 5-bit SGU)
          for (int o=0; o<SGU_OP_PER_CH; o++) {
            DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
            op.ar=((c.value&15)<<1)|1;
            opWrite(c.chan,o,SGU_OP_REG_AR_DR,((op.ar&0x0f)<<SGU_OP2_AR_SHIFT)|(op.dr&SGU_OP2_DR_MASK));
            opWrite(c.chan,o,SGU_OP_REG_OUT_WAVE,(chan[c.chan].state.esfm.op[o].outLvl<<SGU_OP7_OUT_SHIFT)|((op.ar&0x10)?SGU_OP7_AR_MSB_BIT:0)|((op.dr&0x10)?SGU_OP7_DR_MSB_BIT:0)|(op.ws&SGU_OP7_WAVE_MASK));
          }
          break;
        case 0x1: // decay - set DR on all operators (convert 4-bit C64 to 5-bit SGU)
          for (int o=0; o<SGU_OP_PER_CH; o++) {
            DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
            op.dr=((c.value&15)<<1)|1;
            opWrite(c.chan,o,SGU_OP_REG_AR_DR,((op.ar&0x0f)<<SGU_OP2_AR_SHIFT)|(op.dr&SGU_OP2_DR_MASK));
            opWrite(c.chan,o,SGU_OP_REG_OUT_WAVE,(chan[c.chan].state.esfm.op[o].outLvl<<SGU_OP7_OUT_SHIFT)|((op.ar&0x10)?SGU_OP7_AR_MSB_BIT:0)|((op.dr&0x10)?SGU_OP7_DR_MSB_BIT:0)|(op.ws&SGU_OP7_WAVE_MASK));
          }
          break;
        case 0x2: // sustain - set SL on all operators
          for (int o=0; o<SGU_OP_PER_CH; o++) {
            DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
            op.sl=c.value&15;
            opWrite(c.chan,o,SGU_OP_REG_SL_RR,(op.sl<<SGU_OP3_SL_SHIFT)|(op.rr&SGU_OP3_RR_MASK));
          }
          break;
        case 0x3: // release - set RR on all operators
          for (int o=0; o<SGU_OP_PER_CH; o++) {
            DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
            op.rr=c.value&15;
            opWrite(c.chan,o,SGU_OP_REG_SL_RR,(op.sl<<SGU_OP3_SL_SHIFT)|(op.rr&SGU_OP3_RR_MASK));
          }
          break;
        case 0x4: // ring mod - set RING bit on operators (each op rings with previous)
          for (int o=0; o<SGU_OP_PER_CH; o++) {
            DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
            DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
            unsigned char reg6=(op.dam?SGU_OP6_TRMD_BIT:0)|(op.dvb?SGU_OP6_VIBD_BIT:0)|((c.value&1)?SGU_OP6_RING_BIT:0)|(opE.modIn<<SGU_OP6_MOD_SHIFT)|((op.tl>>6)&SGU_OP6_TL_MSB_BIT);
            opWrite(c.chan,o,SGU_OP_REG_MOD,reg6);
          }
          break;
        case 0x5: // sync - set SYNC bit on operators
          for (int o=0; o<SGU_OP_PER_CH; o++) {
            DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
            DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
            unsigned char reg6=(op.dam?SGU_OP6_TRMD_BIT:0)|(op.dvb?SGU_OP6_VIBD_BIT:0)|((c.value&1)?SGU_OP6_SYNC_BIT:0)|(opE.modIn<<SGU_OP6_MOD_SHIFT)|((op.tl>>6)&SGU_OP6_TL_MSB_BIT);
            opWrite(c.chan,o,SGU_OP_REG_MOD,reg6);
          }
          break;
        case 0x6: // filter control (ch3off in C64)
          chan[c.chan].control=(chan[c.chan].control&7)|((c.value&1)<<3);
          writeControl(c.chan);
          break;
        case 0x9: // phase reset
          chan[c.chan].phaseReset=true;
          writeControlUpper(c.chan);
          break;
        case 0xa: // envelope/gate on/off
          chan[c.chan].gate=(c.value&1);
          writeControl(c.chan);
          break;
        case 0xb: // filter on/off - use control bits
          if (c.value&1) {
            chan[c.chan].control|=7; // enable LP+BP+HP
          } else {
            chan[c.chan].control&=~7; // disable all filter modes
          }
          writeControl(c.chan);
          break;
      }
      break;
    // SoundUnit-specific commands
    case DIV_CMD_WAVE: {
      // Map SID wave bits to SGU operators: tri(0)→op0, saw(1)→op1, pulse(2)→op2, noise(3)→op3
      // Each enabled wave gets MOD=0 (no modulation), OUT=7 (full output)
      chan[c.chan].wave=c.value&0xf;
      static const unsigned char sidToSguWave[4]={SGU_WAVE_TRIANGLE, SGU_WAVE_SAWTOOTH, SGU_WAVE_PULSE, SGU_WAVE_NOISE};
      for (int o=0; o<SGU_OP_PER_CH; o++) {
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        bool waveEnabled=(c.value>>o)&1;
        if (waveEnabled) {
          op.tl=0;  // full volume
          op.ws=sidToSguWave[o];
          opE.modIn=0;  // no modulation from previous
          opE.outLvl=7;  // full output
        } else {
          opE.outLvl=0;  // no output
        }
        // Write operator registers
        opWrite(c.chan,o,SGU_OP_REG_TL,(op.tl&SGU_OP1_TL_MASK)|(op.ksl<<SGU_OP1_KSL_SHIFT));
        opWrite(c.chan,o,SGU_OP_REG_MOD,(op.dam?SGU_OP6_TRMD_BIT:0)|(op.dvb?SGU_OP6_VIBD_BIT:0)|(opE.modIn<<SGU_OP6_MOD_SHIFT)|((op.tl>>6)&SGU_OP6_TL_MSB_BIT));
        opWrite(c.chan,o,SGU_OP_REG_OUT_WAVE,(opE.outLvl<<SGU_OP7_OUT_SHIFT)|((op.ar&0x10)?SGU_OP7_AR_MSB_BIT:0)|((op.dr&0x10)?SGU_OP7_DR_MSB_BIT:0)|(op.ws&SGU_OP7_WAVE_MASK));
      }
      writeControl(c.chan);
      break;
    }
    case DIV_CMD_C64_RESONANCE:
      chan[c.chan].res=c.value;
      chWrite(c.chan,SGU1_CHN_RESON,chan[c.chan].res);
      break;
    case DIV_CMD_C64_FILTER_MODE:
      chan[c.chan].control=c.value&15;
      writeControl(c.chan);
      break;
    case DIV_CMD_C64_CUTOFF:
      if (c.value>100) c.value=100;
      chan[c.chan].baseCutoff=((c.value+2)*16383)/102;
      if (!chan[c.chan].std.ex1.has) {
        chan[c.chan].cutoff=chan[c.chan].baseCutoff;
        chWrite(c.chan,SGU1_CHN_CUTOFF_L,chan[c.chan].cutoff&0xff);
        chWrite(c.chan,SGU1_CHN_CUTOFF_H,chan[c.chan].cutoff>>8);
      }
      break;
    case DIV_CMD_C64_FINE_CUTOFF:
      chan[c.chan].baseCutoff=c.value*4;
      if (!chan[c.chan].std.ex1.has) {
        chan[c.chan].cutoff=chan[c.chan].baseCutoff;
        chWrite(c.chan,SGU1_CHN_CUTOFF_L,chan[c.chan].cutoff&0xff);
        chWrite(c.chan,SGU1_CHN_CUTOFF_H,chan[c.chan].cutoff>>8);
      }
      break;
    case DIV_CMD_C64_FINE_DUTY:
      chan[c.chan].duty=c.value&127;
      chan[c.chan].virtual_duty=(unsigned short)chan[c.chan].duty<<5;
      chWrite(c.chan,SGU1_CHN_DUTY,chan[c.chan].duty);
      break;
    case DIV_CMD_C64_AD:
      // Set AR and DR on all operators (convert 4-bit C64 to 5-bit SGU)
      for (int o=0; o<SGU_OP_PER_CH; o++) {
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        op.ar=((c.value>>4)<<1)|1;
        op.dr=((c.value&15)<<1)|1;
        opWrite(c.chan,o,SGU_OP_REG_AR_DR,((op.ar&0x0f)<<SGU_OP2_AR_SHIFT)|(op.dr&SGU_OP2_DR_MASK));
        opWrite(c.chan,o,SGU_OP_REG_OUT_WAVE,(chan[c.chan].state.esfm.op[o].outLvl<<SGU_OP7_OUT_SHIFT)|((op.ar&0x10)?SGU_OP7_AR_MSB_BIT:0)|((op.dr&0x10)?SGU_OP7_DR_MSB_BIT:0)|(op.ws&SGU_OP7_WAVE_MASK));
      }
      break;
    case DIV_CMD_C64_SR:
      // Set SL and RR on all operators
      for (int o=0; o<SGU_OP_PER_CH; o++) {
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        op.sl=c.value>>4;
        op.rr=c.value&15;
        opWrite(c.chan,o,SGU_OP_REG_SL_RR,(op.sl<<SGU_OP3_SL_SHIFT)|(op.rr&SGU_OP3_RR_MASK));
      }
      break;
    case DIV_CMD_SU_SWEEP_PERIOD_LOW: {
      switch (c.value) {
        case 0:
          chan[c.chan].freqSweepP=(chan[c.chan].freqSweepP&0xff00)|c.value2;
          chWrite(c.chan,SGU1_CHN_SWFREQ_SPD_L,chan[c.chan].freqSweepP&0xff);
          break;
        case 1:
          chan[c.chan].volSweepP=(chan[c.chan].volSweepP&0xff00)|c.value2;
          chWrite(c.chan,SGU1_CHN_SWVOL_SPD_L,chan[c.chan].volSweepP&0xff);
          break;
        case 2:
          chan[c.chan].cutSweepP=(chan[c.chan].cutSweepP&0xff00)|c.value2;
          chWrite(c.chan,SGU1_CHN_SWCUT_SPD_L,chan[c.chan].cutSweepP&0xff);
          break;
      }
      break;
    }
    case DIV_CMD_SU_SWEEP_PERIOD_HIGH: {
      switch (c.value) {
        case 0:
          chan[c.chan].freqSweepP=(chan[c.chan].freqSweepP&0xff)|(c.value2<<8);
          chWrite(c.chan,SGU1_CHN_SWFREQ_SPD_H,chan[c.chan].freqSweepP>>8);
          break;
        case 1:
          chan[c.chan].volSweepP=(chan[c.chan].volSweepP&0xff)|(c.value2<<8);
          chWrite(c.chan,SGU1_CHN_SWVOL_SPD_H,chan[c.chan].volSweepP>>8);
          break;
        case 2:
          chan[c.chan].cutSweepP=(chan[c.chan].cutSweepP&0xff)|(c.value2<<8);
          chWrite(c.chan,SGU1_CHN_SWCUT_SPD_H,chan[c.chan].cutSweepP>>8);
          break;
      }
      break;
    }
    case DIV_CMD_SU_SWEEP_BOUND: {
      switch (c.value) {
        case 0:
          chan[c.chan].freqSweepB=c.value2;
          chWrite(c.chan,SGU1_CHN_SWFREQ_BND,chan[c.chan].freqSweepB);
          break;
        case 1:
          chan[c.chan].volSweepB=c.value2;
          chWrite(c.chan,SGU1_CHN_SWVOL_BND,chan[c.chan].volSweepB);
          break;
        case 2:
          chan[c.chan].cutSweepB=c.value2;
          chWrite(c.chan,SGU1_CHN_SWCUT_BND,chan[c.chan].cutSweepB);
          break;
      }
      break;
    }
    case DIV_CMD_SU_SWEEP_ENABLE: {
      switch (c.value) {
        case 0:
          chan[c.chan].freqSweepV=c.value2;
          chan[c.chan].freqSweep=(c.value2>0);
          chWrite(c.chan,SGU1_CHN_SWFREQ_AMT,chan[c.chan].freqSweepV);
          break;
        case 1:
          chan[c.chan].volSweepV=c.value2;
          chan[c.chan].volSweep=(c.value2>0);
          chWrite(c.chan,SGU1_CHN_SWVOL_AMT,chan[c.chan].volSweepV);
          break;
        case 2:
          chan[c.chan].cutSweepV=c.value2;
          chan[c.chan].cutSweep=(c.value2>0);
          chWrite(c.chan,SGU1_CHN_SWCUT_AMT,chan[c.chan].cutSweepV);
          break;
      }
      writeControlUpper(c.chan);
      break;
    }
    case DIV_CMD_SU_SYNC_PERIOD_LOW:
      chan[c.chan].syncTimer=(chan[c.chan].syncTimer&0xff00)|c.value;
      chan[c.chan].timerSync=(chan[c.chan].syncTimer>0);
      chWrite(c.chan,SGU1_CHN_RESTIMER_L,chan[c.chan].syncTimer&0xff);
      chWrite(c.chan,SGU1_CHN_RESTIMER_H,chan[c.chan].syncTimer>>8);
      writeControlUpper(c.chan);
      break;
    case DIV_CMD_SU_SYNC_PERIOD_HIGH:
      chan[c.chan].syncTimer=(chan[c.chan].syncTimer&0xff)|(c.value<<8);
      chan[c.chan].timerSync=(chan[c.chan].syncTimer>0);
      chWrite(c.chan,SGU1_CHN_RESTIMER_L,chan[c.chan].syncTimer&0xff);
      chWrite(c.chan,SGU1_CHN_RESTIMER_H,chan[c.chan].syncTimer>>8);
      writeControlUpper(c.chan);
      break;
    case DIV_CMD_SAMPLE_POS:
      chan[c.chan].hasOffset=c.value;
      chan[c.chan].keyOn=true;
      break;
    case DIV_CMD_C64_PW_SLIDE:
      chan[c.chan].pw_slide=c.value*c.value2;
      break;
    case DIV_CMD_C64_CUTOFF_SLIDE:
      chan[c.chan].cutoff_slide=c.value*c.value2;
      break;
    default:
      break;
  }
  return 1;
}

void DivPlatformSGU::forceIns() {
  for (int i=0; i<8; i++) {
    chan[i].insChanged=true;
    chan[i].freqChanged=true;

    // restore channel attributes
    chWrite(i,0x03,chan[i].pan);
    writeControl(i);
    writeControlUpper(i);
    chWrite(i,0x08,chan[i].duty);
  }
}

void DivPlatformSGU::toggleRegisterDump(bool enable) {
  DivDispatch::toggleRegisterDump(enable);
}

void* DivPlatformSGU::getChanState(int ch) {
  return &chan[ch];
}

DivMacroInt* DivPlatformSGU::getChanMacroInt(int ch) {
  return &chan[ch].std;
}

unsigned short DivPlatformSGU::getPan(int ch) {
  return parent->convertPanLinearToSplit(chan[ch].pan+127,8,255);
}

DivDispatchOscBuffer* DivPlatformSGU::getOscBuffer(int ch) {
  return oscBuf[ch];
}

unsigned char* DivPlatformSGU::getRegisterPool() {
  return reinterpret_cast<unsigned char*>(chip.chan);
}

int DivPlatformSGU::getRegisterPoolSize() {
  return (SGU_CHNS * SGU_REGS_PER_CH);
}

void DivPlatformSGU::reset() {
  while (!writes.empty()) writes.pop();

  SGU_Reset(&chip);

  for (int i=0; i<SGU_CHNS; i++) {
    chan[i]=DivPlatformSGU::Channel();
    chan[i].std.setEngine(parent);

    chan[i].vol=0x3f;
    chan[i].outVol=0x3f;

    chan[i].cutoff_slide=0;
    chan[i].pw_slide=0;

    chan[i].virtual_duty=0x800; // for some reason duty by default is 50%
  }

  oldOut[0]=0;
  oldOut[1]=0;
}

int DivPlatformSGU::getOutputCount() {
  return SGU_AUDIO_CHANNELS;
}

bool DivPlatformSGU::keyOffAffectsArp(int ch) {
  return false;
}

bool DivPlatformSGU::keyOffAffectsPorta(int ch) {
  return false;
}

bool DivPlatformSGU::hasAcquireDirect() {
  return true;
}

bool DivPlatformSGU::getLegacyAlwaysSetVolume() {
  return false;
}

void DivPlatformSGU::notifyInsChange(int ins) {
  for (int i=0; i<SGU_CHNS; i++) {
    if (chan[i].ins==ins) {
      chan[i].insChanged=true;
    }
  }
}

void DivPlatformSGU::notifyInsDeletion(void* ins) {
  for (int i=0; i<SGU_CHNS; i++) {
    chan[i].std.notifyInsDeletion((DivInstrument*)ins);
  }
}

int DivPlatformSGU::mapVelocity(int ch, float vel) {
  const int volMax=MAX(1,dispatch(DivCommand(DIV_CMD_GET_VOLMAX,MAX(ch,0))));
  double attenDb=20*log10(vel); // 20dB/decade for a linear mapping
  double attenUnits=attenDb/0.75; // 0.75dB/unit
  return MAX(0,volMax+attenUnits);
}

void DivPlatformSGU::poke(unsigned int addr, unsigned short val) {
  rWrite(addr,val);
}

void DivPlatformSGU::poke(std::vector<DivRegWrite>& wlist) {
  for (DivRegWrite& i: wlist) rWrite(i.addr,i.val);
}

void DivPlatformSGU::setFlags(const DivConfig& flags) {
  // chipClock is used for frequency calculations (Fclk=1MHz per SID semantics)
  // rate is the actual sample rate (48kHz)
  chipClock=1000000;
  CHECK_CUSTOM_CLOCK;
  rate=SGU_CHIP_CLOCK;
  for (int i=0; i<SGU_CHNS; i++) {
    oscBuf[i]->setRate(rate);
  }
}

const void* DivPlatformSGU::getSampleMem(int index) {
  return (index==0)?chip.pcm:NULL;
}

size_t DivPlatformSGU::getSampleMemCapacity(int index) {
  return (index==0)?SGU_PCM_RAM_SIZE:0;
}

size_t DivPlatformSGU::getSampleMemUsage(int index) {
  return (index==0)?chip.pcm_size:0;
}

bool DivPlatformSGU::isSampleLoaded(int index, int sample) {
  if (index!=0) return false;
  if (sample<0 || sample>255) return false;
  return sampleLoaded[sample];
}

const DivMemoryComposition* DivPlatformSGU::getMemCompo(int index) {
  if (index!=0) return NULL;
  return &memCompo;
}

void DivPlatformSGU::renderSamples(int sysID) {
  memset(chip.pcm,0,SGU_PCM_RAM_SIZE);
  memset(sampleOffSGU,0,256*sizeof(unsigned int));
  memset(sampleLoaded,0,256*sizeof(bool));

  memCompo=DivMemoryComposition();
  memCompo.name="Sample RAM";

  size_t memPos=0;
  for (int i=0; i<parent->song.sampleLen; i++) {
    DivSample* s=parent->song.sample[i];
    if (s->data8==NULL) continue;
    if (!s->renderOn[0][sysID]) {
      sampleOffSGU[i]=0;
      continue;
    }

    int paddedLen=s->length8;
    if (memPos>=SGU_PCM_RAM_SIZE) {
      logW("out of PCM memory for sample %d!",i);
      break;
    }
    if (memPos+paddedLen>=SGU_PCM_RAM_SIZE) {
      memcpy(chip.pcm+memPos,s->data8,SGU_PCM_RAM_SIZE-memPos);
      logW("out of PCM memory for sample %d!",i);
    } else {
      memcpy(chip.pcm+memPos,s->data8,paddedLen);
      sampleLoaded[i]=true;
    }
    sampleOffSGU[i]=memPos;
    memCompo.entries.push_back(DivMemoryEntry(DIV_MEMORY_SAMPLE,"Sample",i,memPos,memPos+paddedLen));
    memPos+=paddedLen;
  }
  chip.pcm_size=memPos;
  sysIDCache=sysID;

  memCompo.used=memPos;
  memCompo.capacity=SGU_PCM_RAM_SIZE;
}

int DivPlatformSGU::init(DivEngine* p, int channels, int sugRate, const DivConfig& flags) {
  parent=p;
  dumpWrites=false;
  skipRegisterWrites=false;
  sysIDCache=0;
  for (int i=0; i<SGU_CHNS; i++) {
    isMuted[i]=false;
    oscBuf[i]=new DivDispatchOscBuffer;
  }
  SGU_Init(&chip, SGU_PCM_RAM_SIZE);
  setFlags(flags);
  reset();

  return SGU_CHNS;
}

void DivPlatformSGU::quit() {
  for (int i=0; i<SGU_CHNS; i++) {
    delete oscBuf[i];
  }
}

bool DivPlatformSGU::hasSoftPan(int ch) {
  return true;
}

// initialization of sample tracking arrays
DivPlatformSGU::DivPlatformSGU() {
  sampleOffSGU=new unsigned int[256];
  sampleLoaded=new bool[256];
}

DivPlatformSGU::~DivPlatformSGU() {
  delete[] sampleOffSGU;
  delete[] sampleLoaded;
}
