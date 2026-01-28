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

#ifndef _DIV_PLATFORM_SGU_H
#define _DIV_PLATFORM_SGU_H

#include "../dispatch.h"
#include "../../fixedQueue.h"
#include "sound/sgu.h"

class DivPlatformSGU: public DivDispatch {
  struct Channel: public SharedChannel<signed char> {
    struct {
      DivInstrumentFM fm;
      DivInstrumentESFM esfm;
    } state;

    int cutoff, baseCutoff;
    unsigned char res, control;
    signed char pan;
    unsigned char duty;
    bool pcm, pcmLoop;
    bool phaseReset, filterPhaseReset, timerSync;
    bool freqSweep, volSweep, cutSweep;
    unsigned short freqSweepP, volSweepP, cutSweepP;
    unsigned char freqSweepB, volSweepB, cutSweepB;
    unsigned char freqSweepV, volSweepV, cutSweepV;
    unsigned short syncTimer;
    int hasOffset, sample;
    bool released;
    short cutoff_slide;
    short pw_slide;
    short virtual_duty;
    unsigned char ringMask;
    unsigned char syncMask;
    bool key;

    Channel():
      SharedChannel<signed char>(127),
      cutoff(0xffff),
      baseCutoff(0xffff),
      res(0),
      control(0),
      pan(0),
      duty(63),
      pcm(false),
      pcmLoop(false),
      phaseReset(false),
      filterPhaseReset(false),
      timerSync(false),
      freqSweep(false),
      volSweep(false),
      cutSweep(false),
      freqSweepP(0),
      volSweepP(0),
      cutSweepP(0),
      freqSweepB(0),
      volSweepB(0),
      cutSweepB(0),
      freqSweepV(0),
      volSweepV(0),
      cutSweepV(0),
      syncTimer(0),
      hasOffset(0),
      sample(-1),
      released(false),
      cutoff_slide(0),
      pw_slide(0),
      virtual_duty(0),
      ringMask(0),
      syncMask(0),
      key(false) {}
  };

  Channel chan[SGU_CHNS];
  DivDispatchOscBuffer* oscBuf[SGU_CHNS];
  bool isMuted[SGU_CHNS];

  struct QueuedWrite {
    unsigned short addr;
    unsigned char val;
    QueuedWrite(): addr(0), val(0) {}
    QueuedWrite(unsigned short a, unsigned char v): addr(a), val(v) {}
  };
  FixedQueue<QueuedWrite,2048> writes;

  SGU* sgu;

  static constexpr int SGU_REG_POOL_SIZE = SGU_REGS_PER_CH * SGU_CHNS;
  unsigned char regPool[SGU_REG_POOL_SIZE];

  unsigned int* sampleOffSGU;
  bool* sampleLoaded;
  signed char* sampleMem;
  size_t sampleMemLen;
  DivMemoryComposition memCompo;
  int sysIDCache;

  void writeControl(int ch);
  void writeControlUpper(int ch);
  void applyOpRegs(int ch, int op);
  void commitState(int ch, DivInstrument* ins);

  friend void putDispatchChip(void*,int);
  friend void putDispatchChan(void*,int,int);

  public:
    void acquire(short** buf, size_t len);
    int dispatch(DivCommand c);
    void* getChanState(int chan);
    DivMacroInt* getChanMacroInt(int ch);
    unsigned short getPan(int ch);
    DivDispatchOscBuffer* getOscBuffer(int chan);
    unsigned char* getRegisterPool();
    int getRegisterPoolSize();
    void reset();
    void forceIns();
    void tick(bool sysTick=true);
    void muteChannel(int ch, bool mute);
    int getOutputCount();
    bool hasSoftPan(int ch);
    bool keyOffAffectsArp(int ch);
    bool keyOffAffectsPorta(int ch);
    void notifyInsDeletion(void* ins);
    void notifyInsChange(int ins);
    void setFlags(const DivConfig& flags);
    void poke(unsigned int addr, unsigned short val);
    void poke(std::vector<DivRegWrite>& wlist);
    const char** getRegisterSheet();
    const void* getSampleMem(int index);
    size_t getSampleMemCapacity(int index);
    size_t getSampleMemUsage(int index);
    bool isSampleLoaded(int index, int sample);
    const DivMemoryComposition* getMemCompo(int index);
    void renderSamples(int sysID);
    int init(DivEngine* parent, int channels, int sugRate, const DivConfig& flags);
    void quit();
    DivPlatformSGU();
    ~DivPlatformSGU();
};

#endif
