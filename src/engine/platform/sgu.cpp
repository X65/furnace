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
#include <string.h>
#include <math.h>

// SGU-1 replayer notes:
// - Entry points: init/reset/tick/acquire wire the core to Furnace's timing/output path.
// - FM vs PCM: commitState picks FM (ESFM-style ops) or PCM (Amiga sample path via SoundUnit-style regs).
// - SID-like pitch: NOTE_FREQUENCY gives freq16 @ 1 MHz; calcFreq keeps freq16 with CHIP_FREQBASE, then write SGU FREQ regs.
// - OPN envelope: AR/DR/SR are 5-bit; MSBs live in op reg7, SR lives in reg4 (applyOpRegs).
// - RING/SYNC: per-operator bits in reg6; masks map op0..3 to the previous operator (op0 uses op3).
// - Global regs: SoundUnit-style flags0/flags1; KEY-ON is flags0 bit0; WAVE bits live per-operator.

#define rWrite(a,v) if (!skipRegisterWrites) {writes.push(QueuedWrite(a,v)); if (dumpWrites) {addWrite(a,v);} if ((a) < SGU_REG_POOL_SIZE) {regPool[(a)]=(v);} }

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

static unsigned char sguC64Wave(const DivInstrumentC64& c64, bool periodicNoise) {
  if (c64.noiseOn) return periodicNoise ? SGU_WAVE_PERIODIC_NOISE : SGU_WAVE_NOISE;
  if (c64.pulseOn) return SGU_WAVE_PULSE;
  if (c64.sawOn) return SGU_WAVE_SAWTOOTH;
  if (c64.triOn) return SGU_WAVE_TRIANGLE;
  return SGU_WAVE_SAWTOOTH;
}

static int sguVolScaleLinear(const DivEngine* parent, int x, int y, int range) {
  if (parent->song.compatFlags.ceilVolumeScaling) {
    return (((x*y)+(range-1))/range);
  }
  return ((x*y)/range);
}

static int sguVolScaleLog(int x, int y, int range) {
  return CLAMP(((x+y)-range),0,range);
}

static int sguVolScaleLogBroken(const DivEngine* parent, int x, int y, int range) {
  if (parent->song.compatFlags.newVolumeScaling) {
    return sguVolScaleLog(x,y,range);
  }
  return sguVolScaleLinear(parent,x,y,range);
}

static unsigned char sguOpllScaleVol(const DivEngine* parent, int chanVol, int macroVol) {
  int base=CLAMP(chanVol,0,127);
  int base15=(base*15+63)/127;
  int macro15=CLAMP(macroVol,0,15);
  int vol15=sguVolScaleLogBroken(parent,base15,macro15,15);
  return (unsigned char)((vol15*127+7)/15);
}

void DivPlatformSGU::writeControl(int ch) {
  // flags0 uses SoundUnit-like layout, but bit 0 is KEY-ON instead of waveform.
  const unsigned char key = chan[ch].key ? 1 : 0;
  const unsigned char flags0 = (key & 1) | (chan[ch].pcm ? (1 << 3) : 0) | ((chan[ch].control & 0x0f) << 4);
  chWrite(ch, SGU1_CHN_FLAGS0, flags0);
}

void DivPlatformSGU::writeControlUpper(int ch) {
  const unsigned char flags1 = (chan[ch].phaseReset ? 1 : 0)
    | (chan[ch].filterPhaseReset ? 2 : 0)
    | (chan[ch].pcmLoop ? 4 : 0)
    | (chan[ch].timerSync ? 8 : 0)
    | (chan[ch].freqSweep ? 16 : 0)
    | (chan[ch].volSweep ? 32 : 0)
    | (chan[ch].cutSweep ? 64 : 0);
  chWrite(ch, SGU1_CHN_FLAGS1, flags1);
  chan[ch].phaseReset=false;
  chan[ch].filterPhaseReset=false;
}

void DivPlatformSGU::applyOpRegs(int ch, int o) {
  DivInstrumentFM::Operator& op=chan[ch].state.fm.op[o];
  DivInstrumentESFM::Operator& opE=chan[ch].state.esfm.op[o];

  const unsigned char tl = op.tl & 0x7f;
  const unsigned char ar = op.ar & 0x1f;
  const unsigned char dr = op.dr & 0x1f;

  const unsigned char reg0 = ((op.am & 1) << 7) | ((op.vib & 1) << 6) | ((opE.fixed & 1) << 5) | (op.mult & 0x0f);
  const unsigned char reg1 = ((op.ksl & 3) << 6) | (tl & 0x3f);
  const unsigned char reg2 = ((ar & 0x0f) << 4) | (dr & 0x0f);
  const unsigned char reg3 = ((op.sl & 0x0f) << 4) | (op.rr & 0x0f);
  const unsigned char reg4 = ((op.dt & 0x07) << 5) | (op.d2r & 0x1f); // SR uses OPN-style 5-bit value

  const unsigned char wpar = 0; // SGU WPAR not yet exposed via Furnace macros
  const unsigned char reg5 = ((opE.delay & 0x07) << 5) | ((op.rs & 0x03) << 3) | (wpar & 0x07);

  // RING/SYNC are per-operator flags; SGU core applies them to the previous operator
  // in the chain (op0 uses op3). We update these on register writes.
  const bool ring = (chan[ch].ringMask >> o) & 1;
  const bool sync = (chan[ch].syncMask >> o) & 1;
  const unsigned char reg6 = ((op.dam & 1) << 7) | ((op.dvb & 1) << 6)
    | (sync ? 0x20 : 0) | (ring ? 0x10 : 0)
    | ((opE.modIn & 0x07) << 1)
    | ((tl >> 6) & 1);

  const unsigned char outLvl = op.enable ? (opE.outLvl & 0x07) : 0;
  // AR/DR are 5-bit OPN-style; MSBs live in reg7.
  const unsigned char reg7 = (outLvl << 5)
    | ((ar & 0x10) ? 0x10 : 0)
    | ((dr & 0x10) ? 0x08 : 0)
    | (op.ws & 0x07);

  opWrite(ch,o,0x00,reg0);
  opWrite(ch,o,0x01,reg1);
  opWrite(ch,o,0x02,reg2);
  opWrite(ch,o,0x03,reg3);
  opWrite(ch,o,0x04,reg4);
  opWrite(ch,o,0x05,reg5);
  opWrite(ch,o,0x06,reg6);
  opWrite(ch,o,0x07,reg7);
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
    case DIV_INS_ESFM:
    case DIV_INS_FM:
    case DIV_INS_OPM:
    case DIV_INS_OPL:
    case DIV_INS_OPLL:
    case DIV_INS_OPZ:
      fm.op[o].ws=oplToSguWaveformMap[fm.op[o].ws & 0x07];
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

  /* OPN2
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
    case DIV_INS_ESFM:
    case DIV_INS_OPL:
      {
        // AR/DR need shifting from 4-bit to 5-bit
        fm.op[o].ar=(unsigned char)(((fm.op[o].ar & 0x0f) << 1) | 1);
        fm.op[o].dr=(unsigned char)(((fm.op[o].dr & 0x0f) << 1) | 1);
        // TL needs shifting from 6-bit to 7-bit
        fm.op[o].tl=(unsigned char)(((fm.op[o].tl & 0x3f) << 1) | 1);
        // Convert EGT flag to D2R rate
        fm.op[o].d2r = fm.op[o].egt ? 0 : 31;
      }
      break;
    case DIV_INS_OPLL:
      {
        // AR/DR need shifting from 4-bit to 5-bit
        fm.op[o].ar=(unsigned char)(((fm.op[o].ar & 0x0f) << 1) | 1);
        fm.op[o].dr=(unsigned char)(((fm.op[o].dr & 0x0f) << 1) | 1);
        // TL: modulator is 6-bit, carrier is 4-bit
        fm.op[o].tl=(o==0)
          ? (unsigned char)(((fm.op[o].tl & 0x3f) << 1) | 1)
          : (unsigned char)(((fm.op[o].tl & 0x0f) << 3) | 0x04);
        // Convert Sustain flag to D2R rate
        fm.op[o].d2r = fm.op[o].sus ? 0 : 31;
        // Convert KSR and apply global AMS/FMS
        fm.op[o].rs=(fm.op[o].ksr&1)?3:0;
        fm.op[o].dam=fm.ams&1;
        fm.op[o].dvb=fm.fms&1;
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
  }

  /* Convert operator algorithms */
  switch (ins->type) {
    case DIV_INS_ESFM:
      // ESFM is native format for SGU - use as-is
      break;
    case DIV_INS_AMIGA:
      // Handled by PCM path above
      break;
    case DIV_INS_FM:
      // OPN-style 4-operator FM
      // Convert algorithm (0-7) to ESFM-style modIn/outLvl routing
      // AR/DR are 5-bit, TL is 7-bit in OPN; already handled by first switch
      {
        esfm=DivInstrumentESFM();
        // OPN algorithm mapping to ESFM operator routing:
        // Algorithm determines which operators output and modulation routing
        switch (fm.alg & 7) {
          case 0: // 1→2→3→4→out (serial)
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=0;
            esfm.op[2].modIn=7;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 1: // (1+2)→3→4→out
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=0;       esfm.op[1].outLvl=0;
            esfm.op[2].modIn=7;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 2: // 1+(2→3)→4→out
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=0;       esfm.op[1].outLvl=0;
            esfm.op[2].modIn=7;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 3: // (1→2)+3→4→out
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=0;
            esfm.op[2].modIn=0;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 4: // (1→2)+(3→4)→out
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=7;
            esfm.op[2].modIn=0;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 5: // 1→(2+3+4)→out
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=7;
            esfm.op[2].modIn=7;       esfm.op[2].outLvl=7;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 6: // (1→2)+3+4→out
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=7;
            esfm.op[2].modIn=0;       esfm.op[2].outLvl=7;
            esfm.op[3].modIn=0;       esfm.op[3].outLvl=7;
            break;
          case 7: // 1+2+3+4→out (additive)
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=7;
            esfm.op[1].modIn=0;       esfm.op[1].outLvl=7;
            esfm.op[2].modIn=0;       esfm.op[2].outLvl=7;
            esfm.op[3].modIn=0;       esfm.op[3].outLvl=7;
            break;
        }
      }
      break;
    case DIV_INS_OPM:
      // OPM-style 4-operator FM (same algorithm structure as OPN)
      // AR/DR are 5-bit, TL is 7-bit; scaling already done in first switch
      {
        esfm=DivInstrumentESFM();
        switch (fm.alg & 7) {
          case 0:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=0;
            esfm.op[2].modIn=7;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 1:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=0;       esfm.op[1].outLvl=0;
            esfm.op[2].modIn=7;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 2:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=0;       esfm.op[1].outLvl=0;
            esfm.op[2].modIn=7;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 3:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=0;
            esfm.op[2].modIn=0;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 4:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=7;
            esfm.op[2].modIn=0;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 5:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=7;
            esfm.op[2].modIn=7;       esfm.op[2].outLvl=7;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 6:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=7;
            esfm.op[2].modIn=0;       esfm.op[2].outLvl=7;
            esfm.op[3].modIn=0;       esfm.op[3].outLvl=7;
            break;
          case 7:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=7;
            esfm.op[1].modIn=0;       esfm.op[1].outLvl=7;
            esfm.op[2].modIn=0;       esfm.op[2].outLvl=7;
            esfm.op[3].modIn=0;       esfm.op[3].outLvl=7;
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
    case DIV_INS_OPZ:
      // OPZ-style 4-operator FM (similar to OPM with fixed frequency support)
      {
        esfm=DivInstrumentESFM();
        // Copy fixed frequency flags
        for (int o=0; o<4; o++) {
          esfm.op[o].fixed=fm.op[o].egt;
        }
        switch (fm.alg & 7) {
          case 0:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=0;
            esfm.op[2].modIn=7;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 1:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=0;       esfm.op[1].outLvl=0;
            esfm.op[2].modIn=7;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 2:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=0;       esfm.op[1].outLvl=0;
            esfm.op[2].modIn=7;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 3:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=0;
            esfm.op[2].modIn=0;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 4:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=7;
            esfm.op[2].modIn=0;       esfm.op[2].outLvl=0;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 5:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=7;
            esfm.op[2].modIn=7;       esfm.op[2].outLvl=7;
            esfm.op[3].modIn=7;       esfm.op[3].outLvl=7;
            break;
          case 6:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=0;
            esfm.op[1].modIn=7;       esfm.op[1].outLvl=7;
            esfm.op[2].modIn=0;       esfm.op[2].outLvl=7;
            esfm.op[3].modIn=0;       esfm.op[3].outLvl=7;
            break;
          case 7:
            esfm.op[0].modIn=fm.fb&7; esfm.op[0].outLvl=7;
            esfm.op[1].modIn=0;       esfm.op[1].outLvl=7;
            esfm.op[2].modIn=0;       esfm.op[2].outLvl=7;
            esfm.op[3].modIn=0;       esfm.op[3].outLvl=7;
            break;
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

        chan[ch].ringMask=0;
        chan[ch].syncMask=0;
      }
      break;
    default:
      break;
  }

  chan[ch].state.fm=fm;
  chan[ch].state.esfm=esfm;

  for (int o=0; o<4; o++) {
    applyOpRegs(ch,o);
  }
}

void DivPlatformSGU::acquire(short** buf, size_t len) {
  int32_t l=0;
  int32_t r=0;

  for (int i=0; i<SGU_CHNS; i++) {
    oscBuf[i]->begin(len);
  }

  for (size_t h=0; h<len; h++) {
    while (!writes.empty()) {
      QueuedWrite w=writes.front();
      SGU_Write(sgu,w.addr,w.val);
      writes.pop();
    }

    SGU_NextSample(sgu,&l,&r);
    buf[0][h]=CLAMP(l,-32768,32767);
    buf[1][h]=CLAMP(r,-32768,32767);

    for (int i=0; i<SGU_CHNS; i++) {
      oscBuf[i]->putSample(h,SGU_GetSample(sgu,i));
    }
  }

  for (int i=0; i<SGU_CHNS; i++) {
    oscBuf[i]->end(len);
  }
}

void DivPlatformSGU::tick(bool sysTick) {
  for (int i=0; i<SGU_CHNS; i++) {
    chan[i].std.next();
    DivInstrument* ins=parent->getIns(chan[i].ins,DIV_INS_ESFM);
    const bool isOpll=(ins->type==DIV_INS_OPLL);
    const bool isAmiga=(ins->type==DIV_INS_AMIGA || ins->amiga.useSample);

    if (sysTick) {
      if (chan[i].pw_slide!=0) {
        chan[i].virtual_duty-=chan[i].pw_slide;
        chan[i].virtual_duty=CLAMP(chan[i].virtual_duty,0,0xfff);
        chan[i].duty=chan[i].virtual_duty>>5;
        chWrite(i,SGU1_CHN_DUTY,chan[i].duty);
      }
      if (chan[i].cutoff_slide!=0) {
        chan[i].cutoff+=chan[i].cutoff_slide*16;
        chan[i].cutoff=CLAMP(chan[i].cutoff,0,0xffff);
        chWrite(i,SGU1_CHN_CUTOFF_L,chan[i].cutoff&0xff);
        chWrite(i,SGU1_CHN_CUTOFF_H,chan[i].cutoff>>8);
      }
    }

    if (chan[i].std.vol.had) {
      if (isAmiga) {
        chan[i].outVol=((chan[i].vol&127)*MIN(64,chan[i].std.vol.val))>>6;
      } else if (isOpll) {
        chan[i].outVol=sguOpllScaleVol(parent,chan[i].vol,chan[i].std.vol.val);
      } else {
        chan[i].outVol=((chan[i].vol&127)*MIN(127,chan[i].std.vol.val))>>7;
      }
      chWrite(i,SGU1_CHN_VOL,chan[i].outVol);
    }

    if (NEW_ARP_STRAT) {
      chan[i].handleArp();
    } else if (chan[i].std.arp.had) {
      if (!chan[i].inPorta) {
        // baseFreq is SID-style freq16 using 1 MHz clock
        chan[i].baseFreq=NOTE_FREQUENCY(parent->calcArp(chan[i].note,chan[i].std.arp.val));
      }
      chan[i].freqChanged=true;
    }

    if (chan[i].std.duty.had) {
      chan[i].duty=chan[i].std.duty.val&127;
      chan[i].virtual_duty=(unsigned short)chan[i].duty<<5;
      chWrite(i,SGU1_CHN_DUTY,chan[i].duty);
    }

    if (chan[i].std.wave.had) {
      // WAVE bits live in per-operator regs; channel wave macro maps to carrier.
      chan[i].state.fm.op[3].ws=chan[i].std.wave.val&7;
      applyOpRegs(i,3);
    }

    if (chan[i].std.phaseReset.had) {
      chan[i].phaseReset=chan[i].std.phaseReset.val;
      writeControlUpper(i);
    }

    if (chan[i].std.panL.had) {
      chan[i].pan=chan[i].std.panL.val;
      chWrite(i,SGU1_CHN_PAN,chan[i].pan);
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

    if (chan[i].std.ex1.had) {
      chan[i].cutoff=((chan[i].std.ex1.val&0x3fff)*chan[i].baseCutoff)/0x3fff;
      chWrite(i,SGU1_CHN_CUTOFF_L,chan[i].cutoff&0xff);
      chWrite(i,SGU1_CHN_CUTOFF_H,chan[i].cutoff>>8);
    }

    if (chan[i].std.ex2.had) {
      chan[i].res=chan[i].std.ex2.val;
      chWrite(i,SGU1_CHN_RESON,chan[i].res);
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

    if (chan[i].std.fb.had) {
      // Feedback maps to operator 1 MOD depth (op0 feedback) in SGU.
      chan[i].state.esfm.op[0].modIn=chan[i].std.fb.val&7;
      applyOpRegs(i,0);
    }

    if (isOpll && (chan[i].std.fms.had || chan[i].std.ams.had)) {
      if (chan[i].std.fms.had) {
        chan[i].state.fm.fms=chan[i].std.fms.val&1;
      }
      if (chan[i].std.ams.had) {
        chan[i].state.fm.ams=chan[i].std.ams.val&1;
      }
      for (int o=0; o<2; o++) {
        chan[i].state.fm.op[o].dvb=chan[i].state.fm.fms&1;
        chan[i].state.fm.op[o].dam=chan[i].state.fm.ams&1;
        applyOpRegs(i,o);
      }
    }

    for (int o=0; o<4; o++) {
      DivInstrumentFM::Operator& op=chan[i].state.fm.op[o];
      DivInstrumentESFM::Operator& opE=chan[i].state.esfm.op[o];
      DivMacroInt::IntOp& m=chan[i].std.op[o];
      bool opDirty=false;

      if (m.am.had) { op.am=m.am.val; opDirty=true; }
      if (m.vib.had) { op.vib=m.vib.val; opDirty=true; }
      if (m.mult.had) { op.mult=m.mult.val; opDirty=true; }
      if (m.tl.had) {
        if (isOpll) {
          op.tl=(o==0)
            ? (unsigned char)(((m.tl.val & 0x3f) << 1) | 1)
            : (unsigned char)(((m.tl.val & 0x0f) << 3) | 0x04);
        } else {
          op.tl=m.tl.val;
        }
        opDirty=true;
      }
      if (m.ar.had) {
        op.ar=isOpll ? (unsigned char)(((m.ar.val & 0x0f) << 1) | 1) : m.ar.val;
        opDirty=true;
      }
      if (m.dr.had) {
        op.dr=isOpll ? (unsigned char)(((m.dr.val & 0x0f) << 1) | 1) : m.dr.val;
        opDirty=true;
      }
      if (m.sl.had) { op.sl=m.sl.val; opDirty=true; }
      if (m.rr.had) {
        op.rr=m.rr.val;
        if (isOpll) {
          op.d2r=(op.ssgEnv & 8) ? 0 : (unsigned char)(((op.rr & 0x0f) << 1) | 1);
        }
        opDirty=true;
      }
      if (m.d2r.had && !isOpll) { op.d2r=m.d2r.val; opDirty=true; }
      if (m.dt.had && !isOpll) { op.dt=m.dt.val; opDirty=true; }
      if (m.rs.had) { op.rs=m.rs.val&3; opDirty=true; }
      if (!m.rs.had && m.ksr.had) {
        op.rs=m.ksr.val&1;
        op.ksr=m.ksr.val&1;
        opDirty=true;
      }
      if (m.ksl.had) { op.ksl=m.ksl.val; opDirty=true; }
      if (m.dam.had) { op.dam=m.dam.val; opDirty=true; }
      if (m.dvb.had) { op.dvb=m.dvb.val; opDirty=true; }
      if (m.ws.had && !isOpll) { op.ws=m.ws.val; opDirty=true; }
      if (m.dt2.had && !isOpll) { opE.delay=m.dt2.val; opDirty=true; }
      if (m.egt.had) {
        if (isOpll) {
          op.ssgEnv=(op.ssgEnv&7)|((m.egt.val&1)<<3);
          op.d2r=(op.ssgEnv & 8) ? 0 : (unsigned char)(((op.rr & 0x0f) << 1) | 1);
        } else {
          opE.outLvl=m.egt.val;
        }
        opDirty=true;
      }
      if (m.ssg.had && !isOpll) { opE.modIn=m.ssg.val; opDirty=true; }

      if (opDirty) {
        applyOpRegs(i,o);
      }
    }

    if (chan[i].freqChanged || chan[i].keyOn || chan[i].keyOff) {
      // Key-off first (before frequency writes) to create edge transition
      if (chan[i].keyOn || chan[i].keyOff) {
        chan[i].key=false;
        writeControl(i);
        if (chan[i].keyOff && chan[i].pcm) {
          chWrite(i,SGU1_CHN_VOL,0);
        }
        chan[i].keyOff=false;
      }

      // baseFreq starts as a semitone note (Furnace pitch table) and becomes SID-style freq16
      // via NOTE_FREQUENCY using a 1 MHz clock. calcFreq keeps everything in freq16, and we
      // write the result directly to SGU channel .freq (SID semantics).
      chan[i].freq=parent->calcFreq(chan[i].baseFreq,chan[i].pitch,
        chan[i].fixedArp?chan[i].baseNoteOverride:chan[i].arpOff,
        chan[i].fixedArp,false,8,chan[i].pitch2,chipClock,CHIP_FREQBASE);

      if (chan[i].pcm) {
        // PCM pitch follows the SoundUnit-style sample playback mapping.
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

      chWrite(i,SGU1_CHN_FREQ_L,chan[i].freq&0xff);
      chWrite(i,SGU1_CHN_FREQ_H,chan[i].freq>>8);

      // Key-on last (after frequency writes) to start new ADSR cycle
      if (chan[i].keyOn) {
        chan[i].key=true;
        writeControl(i);
      }

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
            writeControlUpper(i);
          }
        }
      }

      if (chan[i].keyOn) chan[i].keyOn=false;
      chan[i].freqChanged=false;
    }
  }
}

int DivPlatformSGU::dispatch(DivCommand c) {
  switch (c.cmd) {
    case DIV_CMD_NOTE_ON: {
      DivInstrument* ins=parent->getIns(chan[c.chan].ins,DIV_INS_ESFM);

      chan[c.chan].key=true;
      chan[c.chan].macroInit(ins);
      if (!chan[c.chan].std.vol.will) {
        if (ins->type==DIV_INS_OPLL) {
          chan[c.chan].outVol=sguOpllScaleVol(parent,chan[c.chan].vol,15);
        } else {
          chan[c.chan].outVol=chan[c.chan].vol;
        }
      }

      commitState(c.chan,ins);
      chan[c.chan].insChanged=false;

      chWrite(c.chan,SGU1_CHN_VOL,chan[c.chan].outVol);

      chan[c.chan].pcm=(ins->type==DIV_INS_AMIGA || ins->amiga.useSample);
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

      if (c.value!=DIV_NOTE_NULL) {
        // Input pitch is a semitone note index; NOTE_FREQUENCY converts to SID-like freq16 @ 1 MHz.
        chan[c.chan].baseFreq=NOTE_FREQUENCY(c.value);
        chan[c.chan].note=c.value;
        chan[c.chan].freqChanged=true;
      }

      chan[c.chan].active=true;
      chan[c.chan].keyOn=true;
      chan[c.chan].released=false;
      chan[c.chan].insChanged=false;
      chan[c.chan].keyOff=false;
      break;
    }
    case DIV_CMD_NOTE_OFF:
      chan[c.chan].key=false;
      chan[c.chan].keyOff=true;
      chan[c.chan].keyOn=false;
      break;
    case DIV_CMD_NOTE_OFF_ENV:
      chan[c.chan].key=false;
      chan[c.chan].keyOff=true;
      chan[c.chan].keyOn=false;
      chan[c.chan].std.release();
      break;
    case DIV_CMD_ENV_RELEASE:
      chan[c.chan].std.release();
      chan[c.chan].released=true;
      break;
    case DIV_CMD_INSTRUMENT:
      if (chan[c.chan].ins!=c.value || c.value2==1) {
        chan[c.chan].insChanged=true;
      }
      chan[c.chan].ins=c.value;
      break;
    case DIV_CMD_VOLUME:
      if (chan[c.chan].vol!=c.value) {
        chan[c.chan].vol=c.value;
        if (!chan[c.chan].std.vol.has) {
          DivInstrument* ins=parent->getIns(chan[c.chan].ins,DIV_INS_ESFM);
          if (ins->type==DIV_INS_OPLL) {
            chan[c.chan].outVol=sguOpllScaleVol(parent,chan[c.chan].vol,15);
          } else {
            chan[c.chan].outVol=c.value;
          }
          chWrite(c.chan,SGU1_CHN_VOL,chan[c.chan].outVol);
        }
      }
      break;
    case DIV_CMD_GET_VOLUME:
      if (chan[c.chan].std.vol.has) {
        return chan[c.chan].vol;
      }
      return chan[c.chan].outVol;
      break;
    case DIV_CMD_PITCH:
      chan[c.chan].pitch=c.value;
      chan[c.chan].freqChanged=true;
      break;
    case DIV_CMD_WAVE:
      // Map to carrier waveform.
      chan[c.chan].state.fm.op[3].ws=c.value&7;
      applyOpRegs(c.chan,3);
      break;
    case DIV_CMD_STD_NOISE_MODE:
      chan[c.chan].duty=c.value&127;
      chan[c.chan].virtual_duty=(unsigned short)chan[c.chan].duty << 5;
      chWrite(c.chan,SGU1_CHN_DUTY,chan[c.chan].duty);
      break;
    case DIV_CMD_C64_RESONANCE:
      chan[c.chan].res=c.value;
      chWrite(c.chan,SGU1_CHN_RESON,chan[c.chan].res);
      break;
    case DIV_CMD_C64_FILTER_MODE:
      chan[c.chan].control=c.value&15;
      writeControl(c.chan);
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
    case DIV_CMD_C64_FINE_CUTOFF:
      chan[c.chan].baseCutoff=c.value<<4;
      if (!chan[c.chan].std.ex1.has) {
        chan[c.chan].cutoff=chan[c.chan].baseCutoff;
        chWrite(c.chan,SGU1_CHN_CUTOFF_L,chan[c.chan].cutoff&0xff);
        chWrite(c.chan,SGU1_CHN_CUTOFF_H,chan[c.chan].cutoff>>8);
      }
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
    case DIV_CMD_PANNING:
      chan[c.chan].pan=parent->convertPanSplitToLinearLR(c.value,c.value2,254)-127;
      chWrite(c.chan,SGU1_CHN_PAN,chan[c.chan].pan);
      break;
    case DIV_CMD_SAMPLE_POS:
      chan[c.chan].hasOffset=c.value;
      chan[c.chan].keyOn=true;
      break;
    case DIV_CMD_LEGATO:
      chan[c.chan].baseFreq=NOTE_FREQUENCY(c.value+chan[c.chan].sampleNoteDelta+((HACKY_LEGATO_MESS)?(chan[c.chan].std.arp.val):(0)));
      chan[c.chan].freqChanged=true;
      chan[c.chan].note=c.value;
      break;
    case DIV_CMD_PRE_PORTA:
      if (chan[c.chan].active && !chan[c.chan].keyOff && c.value) {
        chan[c.chan].inPorta=true;
      } else {
        chan[c.chan].inPorta=false;
      }
      chan[c.chan].portaPause=(c.value2==1);
      break;
    case DIV_CMD_C64_PW_SLIDE:
      chan[c.chan].pw_slide=c.value*c.value2;
      break;
    case DIV_CMD_C64_CUTOFF_SLIDE:
      chan[c.chan].cutoff_slide=c.value*c.value2;
      break;
    case DIV_CMD_FM_AM:
    case DIV_CMD_FM_AR:
    case DIV_CMD_FM_DR:
    case DIV_CMD_FM_SL:
    case DIV_CMD_FM_D2R:
    case DIV_CMD_FM_RR:
    case DIV_CMD_FM_DT:
    case DIV_CMD_FM_DT2:
    case DIV_CMD_FM_RS:
    case DIV_CMD_FM_KSR:
    case DIV_CMD_FM_VIB:
    case DIV_CMD_FM_SUS:
    case DIV_CMD_FM_WS:
    case DIV_CMD_FM_SSG:
    case DIV_CMD_FM_REV:
    case DIV_CMD_FM_EG_SHIFT:
    case DIV_CMD_FM_TL:
    case DIV_CMD_FM_MULT: {
      const int oStart=(c.value<0)?0:c.value;
      const int oEnd=(c.value<0)?4:(c.value+1);
      for (int o=oStart; o<oEnd && o<4; o++) {
        DivInstrumentFM::Operator& op=chan[c.chan].state.fm.op[o];
        DivInstrumentESFM::Operator& opE=chan[c.chan].state.esfm.op[o];
        bool opDirty=false;
        switch (c.cmd) {
          case DIV_CMD_FM_AM: op.am=c.value2&1; opDirty=true; break;
          case DIV_CMD_FM_VIB: op.vib=c.value2&1; opDirty=true; break;
          case DIV_CMD_FM_AR: op.ar=c.value2&31; opDirty=true; break;
          case DIV_CMD_FM_DR: op.dr=c.value2&31; opDirty=true; break;
          case DIV_CMD_FM_SL: op.sl=c.value2&15; opDirty=true; break;
          case DIV_CMD_FM_D2R: op.d2r=c.value2&31; opDirty=true; break;
          case DIV_CMD_FM_RR: op.rr=c.value2&15; opDirty=true; break;
          case DIV_CMD_FM_DT: op.dt=c.value2&7; opDirty=true; break;
          case DIV_CMD_FM_DT2: opE.delay=c.value2&7; opDirty=true; break;
          case DIV_CMD_FM_RS: op.rs=c.value2&3; opDirty=true; break;
          case DIV_CMD_FM_KSR: op.rs=c.value2&1; opDirty=true; break;
          case DIV_CMD_FM_SUS: op.sus=c.value2&1; break;
          case DIV_CMD_FM_WS: op.ws=c.value2&7; opDirty=true; break;
          case DIV_CMD_FM_SSG: opE.modIn=c.value2&7; opDirty=true; break;
          case DIV_CMD_FM_TL: op.tl=c.value2&127; opDirty=true; break;
          case DIV_CMD_FM_MULT: op.mult=c.value2&15; opDirty=true; break;
          case DIV_CMD_FM_REV:
          case DIV_CMD_FM_EG_SHIFT:
            break;
          default:
            break;
        }
        if (opDirty) applyOpRegs(c.chan,o);
      }
      break;
    }
    case DIV_CMD_FM_FB:
      chan[c.chan].state.esfm.op[0].modIn=c.value&7;
      applyOpRegs(c.chan,0);
      break;
    case DIV_CMD_FM_AM_DEPTH: {
      if (c.value<0) {
        for (int o=0; o<4; o++) {
          chan[c.chan].state.fm.op[o].dam=c.value2&1;
          applyOpRegs(c.chan,o);
        }
      } else if (c.value<4) {
        chan[c.chan].state.fm.op[c.value].dam=c.value2&1;
        applyOpRegs(c.chan,c.value);
      }
      break;
    }
    case DIV_CMD_FM_PM_DEPTH: {
      if (c.value<0) {
        for (int o=0; o<4; o++) {
          chan[c.chan].state.fm.op[o].dvb=c.value2&1;
          applyOpRegs(c.chan,o);
        }
      } else if (c.value<4) {
        chan[c.chan].state.fm.op[c.value].dvb=c.value2&1;
        applyOpRegs(c.chan,c.value);
      }
      break;
    }
    case DIV_CMD_FM_FIXFREQ: {
      if (c.value<0) {
        for (int o=0; o<4; o++) {
          chan[c.chan].state.esfm.op[o].fixed=(c.value2>0);
          applyOpRegs(c.chan,o);
        }
      } else if (c.value<4) {
        chan[c.chan].state.esfm.op[c.value].fixed=(c.value2>0);
        applyOpRegs(c.chan,c.value);
      }
      break;
    }
    case DIV_CMD_ESFM_OUTLVL: {
      if (c.value<0) {
        for (int o=0; o<4; o++) {
          chan[c.chan].state.esfm.op[o].outLvl=c.value2&7;
          applyOpRegs(c.chan,o);
        }
      } else if (c.value<4) {
        chan[c.chan].state.esfm.op[c.value].outLvl=c.value2&7;
        applyOpRegs(c.chan,c.value);
      }
      break;
    }
    case DIV_CMD_ESFM_MODIN: {
      if (c.value<0) {
        for (int o=0; o<4; o++) {
          chan[c.chan].state.esfm.op[o].modIn=c.value2&7;
          applyOpRegs(c.chan,o);
        }
      } else if (c.value<4) {
        chan[c.chan].state.esfm.op[c.value].modIn=c.value2&7;
        applyOpRegs(c.chan,c.value);
      }
      break;
    }
    case DIV_CMD_ESFM_ENV_DELAY: {
      if (c.value<0) {
        for (int o=0; o<4; o++) {
          chan[c.chan].state.esfm.op[o].delay=c.value2&7;
          applyOpRegs(c.chan,o);
        }
      } else if (c.value<4) {
        chan[c.chan].state.esfm.op[c.value].delay=c.value2&7;
        applyOpRegs(c.chan,c.value);
      }
      break;
    }
    case DIV_CMD_C64_EXTENDED:
      switch (c.value>>4) {
        case 4: // RING mask
          // Bits 0-3 map to operators 1-4 (op0 uses op3 as previous per SGU core).
          chan[c.chan].ringMask=c.value&0x0f;
          for (int o=0; o<4; o++) applyOpRegs(c.chan,o);
          break;
        case 5: // SYNC mask
          // Bits 0-3 map to operators 1-4 (op0 uses op3 as previous per SGU core).
          chan[c.chan].syncMask=c.value&0x0f;
          for (int o=0; o<4; o++) applyOpRegs(c.chan,o);
          break;
        default:
          break;
      }
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
    default:
      break;
  }
  return 1;
}

void DivPlatformSGU::muteChannel(int ch, bool mute) {
  isMuted[ch]=mute;
  sgu->muted[ch]=mute;
}

void DivPlatformSGU::forceIns() {
  for (int i=0; i<SGU_CHNS; i++) {
    chan[i].insChanged=true;
    chan[i].freqChanged=true;

    chWrite(i,SGU1_CHN_PAN,chan[i].pan);
    writeControl(i);
    writeControlUpper(i);
    chWrite(i,SGU1_CHN_DUTY,chan[i].duty);
    chWrite(i,SGU1_CHN_CUTOFF_L,chan[i].cutoff&0xff);
    chWrite(i,SGU1_CHN_CUTOFF_H,chan[i].cutoff>>8);
    chWrite(i,SGU1_CHN_RESON,chan[i].res);

    for (int o=0; o<4; o++) {
      applyOpRegs(i,o);
    }
  }
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
  return regPool;
}

int DivPlatformSGU::getRegisterPoolSize() {
  return SGU_REG_POOL_SIZE;
}

void DivPlatformSGU::reset() {
  writes.clear();
  SGU_Reset(sgu);

  for (int i=0; i<SGU_REG_POOL_SIZE; i++) {
    regPool[i]=0;
  }

  for (int i=0; i<SGU_CHNS; i++) {
    chan[i]=DivPlatformSGU::Channel();
    chan[i].std.setEngine(parent);
    sgu->muted[i]=false;

    chWrite(i,SGU1_CHN_VOL,chan[i].outVol);
    chWrite(i,SGU1_CHN_PAN,chan[i].pan);
    chWrite(i,SGU1_CHN_DUTY,chan[i].duty);
    chWrite(i,SGU1_CHN_CUTOFF_L,chan[i].cutoff&0xff);
    chWrite(i,SGU1_CHN_CUTOFF_H,chan[i].cutoff>>8);
    chWrite(i,SGU1_CHN_RESON,chan[i].res);
    writeControl(i);
    writeControlUpper(i);

    for (int o=0; o<4; o++) {
      applyOpRegs(i,o);
    }
  }

  // copy sample memory into SGU RAM
  if (sampleMem) {
    memcpy(sgu->pcm,sampleMem,SGU_PCM_RAM_SIZE);
  }
}

int DivPlatformSGU::getOutputCount() {
  return 2;
}

bool DivPlatformSGU::hasSoftPan(int ch) {
  return true;
}

bool DivPlatformSGU::keyOffAffectsArp(int ch) {
  return chan[ch].pcm;
}

bool DivPlatformSGU::keyOffAffectsPorta(int ch) {
  return chan[ch].pcm;
}

void DivPlatformSGU::notifyInsDeletion(void* ins) {
  for (int i=0; i<SGU_CHNS; i++) {
    chan[i].std.notifyInsDeletion((DivInstrument*)ins);
  }
}

void DivPlatformSGU::notifyInsChange(int ins) {
  for (int i=0; i<SGU_CHNS; i++) {
    if (chan[i].ins==ins) {
      chan[i].insChanged=true;
    }
  }
}

void DivPlatformSGU::setFlags(const DivConfig& flags) {
  chipClock=1000000.0;
  CHECK_CUSTOM_CLOCK;
  rate=SGU_CHIP_CLOCK;
  for (int i=0; i<SGU_CHNS; i++) {
    oscBuf[i]->setRate(rate);
  }
  renderSamples(sysIDCache);
}

void DivPlatformSGU::poke(unsigned int addr, unsigned short val) {
  rWrite(addr,val);
}

void DivPlatformSGU::poke(std::vector<DivRegWrite>& wlist) {
  for (DivRegWrite& i: wlist) rWrite(i.addr,i.val);
}

const void* DivPlatformSGU::getSampleMem(int index) {
  return (index==0)?sampleMem:NULL;
}

size_t DivPlatformSGU::getSampleMemCapacity(int index) {
  return (index==0)?SGU_PCM_RAM_SIZE:0;
}

size_t DivPlatformSGU::getSampleMemUsage(int index) {
  return (index==0)?sampleMemLen:0;
}

bool DivPlatformSGU::isSampleLoaded(int index, int sample) {
  if (index!=0) return false;
  if (sample<0 || sample>32767) return false;
  return sampleLoaded[sample];
}

const DivMemoryComposition* DivPlatformSGU::getMemCompo(int index) {
  if (index!=0) return NULL;
  return &memCompo;
}

void DivPlatformSGU::renderSamples(int sysID) {
  memset(sampleMem,0,SGU_PCM_RAM_SIZE);
  memset(sampleOffSGU,0,32768*sizeof(unsigned int));
  memset(sampleLoaded,0,32768*sizeof(bool));

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
    if (memPos>=getSampleMemCapacity(0)) {
      logW("out of PCM memory for sample %d!",i);
      break;
    }
    if (memPos+paddedLen>=getSampleMemCapacity(0)) {
      memcpy(sampleMem+memPos,s->data8,getSampleMemCapacity(0)-memPos);
      logW("out of PCM memory for sample %d!",i);
    } else {
      memcpy(sampleMem+memPos,s->data8,paddedLen);
      sampleLoaded[i]=true;
    }
    sampleOffSGU[i]=memPos;
    memCompo.entries.push_back(DivMemoryEntry(DIV_MEMORY_SAMPLE,"Sample",i,memPos,memPos+paddedLen));
    memPos+=paddedLen;
  }
  sampleMemLen=memPos;
  sysIDCache=sysID;

  memcpy(sgu->pcm,sampleMem,SGU_PCM_RAM_SIZE);

  memCompo.used=sampleMemLen;
  memCompo.capacity=SGU_PCM_RAM_SIZE;
}

int DivPlatformSGU::init(DivEngine* p, int channels, int sugRate, const DivConfig& flags) {
  parent=p;
  dumpWrites=false;
  skipRegisterWrites=false;

  for (int i=0; i<SGU_CHNS; i++) {
    isMuted[i]=false;
    oscBuf[i]=new DivDispatchOscBuffer;
  }

  sgu=new SGU();
  sampleMem=new signed char[SGU_PCM_RAM_SIZE];
  memset(sampleMem,0,SGU_PCM_RAM_SIZE);

  sysIDCache=0;
  SGU_Init(sgu,SGU_PCM_RAM_SIZE);
  setFlags(flags);
  reset();
  return SGU_CHNS;
}

void DivPlatformSGU::quit() {
  for (int i=0; i<SGU_CHNS; i++) {
    delete oscBuf[i];
  }
  delete sgu;
  delete[] sampleMem;
}

DivPlatformSGU::DivPlatformSGU() {
  sampleOffSGU=new unsigned int[32768];
  sampleLoaded=new bool[32768];
}

DivPlatformSGU::~DivPlatformSGU() {
  delete[] sampleOffSGU;
  delete[] sampleLoaded;
}
