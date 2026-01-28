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

#include "gui.h"
#include <imgui.h>

namespace {
constexpr int kEsfmOpRegs = 8;
constexpr int kEsfmChanStride = 32;
constexpr int kSguOpRegs = 8;
constexpr int kSguChanStride = 64;

int findSystemIndex(const DivSong& song, DivSystem sys) {
  for (int i = 0; i < song.systemLen; i++) {
    if (song.system[i] == sys) return i;
  }
  return -1;
}

void appendHexByte(String& out, unsigned char v) {
  static const char* hex = "0123456789ABCDEF";
  out.push_back(hex[(v >> 4) & 0xF]);
  out.push_back(hex[v & 0xF]);
}

void appendOpBytes(String& out, const unsigned char* pool, int size, int base, int count) {
  for (int i = 0; i < count; i++) {
    if (i) out.push_back(' ');
    int idx = base + i;
    if (pool && idx >= 0 && idx < size) {
      appendHexByte(out, pool[idx]);
    } else {
      out.append("??");
    }
  }
}

String buildSguEsfmOpCompare(const unsigned char* sguPool, int sguSize,
                             const unsigned char* esfmPool, int esfmSize,
                             int sguChan, int esfmChan) {
  String out;
  out.reserve(512);
  out += "OP  | SGU r0 r1 r2 r3 r4 r5 r6 r7 | ESFM r0 r1 r2 r3 r4 r5 r6 r7\n";
  out += "----+--------------------------------+--------------------------------\n";
  for (int op = 0; op < 4; op++) {
    out += "OP";
    out.push_back('0' + op);
    out += " |     ";
    int sguBase = sguChan * kSguChanStride + op * kSguOpRegs;
    appendOpBytes(out, sguPool, sguSize, sguBase, kSguOpRegs);
    out += " |      ";
    int esfmBase = esfmChan * kEsfmChanStride + op * kEsfmOpRegs;
    appendOpBytes(out, esfmPool, esfmSize, esfmBase, kEsfmOpRegs);
    out.push_back('\n');
  }
  return out;
}
} // namespace

void FurnaceGUI::drawRegView() {
  if (nextWindow==GUI_WINDOW_REGISTER_VIEW) {
    channelsOpen=true;
    ImGui::SetNextWindowFocus();
    nextWindow=GUI_WINDOW_NOTHING;
  }
  if (!regViewOpen) return;
  if (ImGui::Begin("Register View",&regViewOpen,globalWinFlags,_("Register View"))) {
    if (ImGui::CollapsingHeader(_("SGU/ESFM ch0 operator compare"), ImGuiTreeNodeFlags_DefaultOpen)) {
      static String sguEsfmDump;
      const int sguSys = findSystemIndex(e->song, DIV_SYSTEM_SGU);
      const int esfmSys = findSystemIndex(e->song, DIV_SYSTEM_ESFM);
      if (sguSys < 0 || esfmSys < 0) {
        ImGui::TextUnformatted(_("Add both SGU-1 and ESFM chips to compare register dumps."));
      } else {
        int sguSize = 0;
        int esfmSize = 0;
        int sguDepth = 8;
        int esfmDepth = 8;
        unsigned char* sguPool = e->getRegisterPool(sguSys, sguSize, sguDepth);
        unsigned char* esfmPool = e->getRegisterPool(esfmSys, esfmSize, esfmDepth);
        if (sguPool == NULL || esfmPool == NULL || sguDepth != 8 || esfmDepth != 8) {
          ImGui::TextUnformatted(_("Register pool unavailable for SGU/ESFM."));
        } else {
          if (ImGui::Button(_("Refresh dump"))) {
            sguEsfmDump = buildSguEsfmOpCompare(sguPool, sguSize, esfmPool, esfmSize, 0, 0);
          }
          ImGui::SameLine();
          ImGui::BeginDisabled(sguEsfmDump.empty());
          if (ImGui::Button(_("Copy dump"))) {
            ImGui::SetClipboardText(sguEsfmDump.c_str());
          }
          ImGui::EndDisabled();
          ImGui::SameLine();
          ImGui::Text(_("(SGU idx %d, ESFM idx %d)"), sguSys + 1, esfmSys + 1);
          ImGui::BeginChild("SguEsfmDump", ImVec2(0.0f, 120.0f * dpiScale), true);
          ImGui::PushFont(patFont);
          ImGui::TextUnformatted(sguEsfmDump.empty() ? _("(click Refresh dump)") : sguEsfmDump.c_str());
          ImGui::PopFont();
          ImGui::EndChild();
        }
      }
    }

    for (int i=0; i<e->song.systemLen; i++) {
      ImGui::Text("%d. %s",i+1,getSystemName(e->song.system[i]));
      int size=0;
      int depth=8;
      unsigned char* regPool=e->getRegisterPool(i,size,depth);
      unsigned short* regPoolW=(unsigned short*)regPool;
      if (regPool==NULL) {
        ImGui::Text(_("- no register pool available"));
      } else {
        ImGui::PushFont(patFont);
        if (ImGui::BeginTable("Memory",17)) {
          float widthOne=ImGui::CalcTextSize("0").x;
          if (size>0xfff) { // no im got gonna put some clamped log formula instead
            ImGui::TableSetupColumn("addr",ImGuiTableColumnFlags_WidthFixed, widthOne*4.0f);
          } else if (size>0xff) {
            ImGui::TableSetupColumn("addr",ImGuiTableColumnFlags_WidthFixed, widthOne*3.0f);
          } else {
            ImGui::TableSetupColumn("addr",ImGuiTableColumnFlags_WidthFixed, widthOne*2.0f);
          }
          
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          for (int i=0; i<16; i++) {
            ImGui::TableNextColumn();
            ImGui::TextColored(uiColors[GUI_COLOR_PATTERN_ROW_INDEX]," %X",i);
          }
          for (int i=0; i<=((size-1)>>4); i++) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextColored(uiColors[GUI_COLOR_PATTERN_ROW_INDEX],"%.2X",i*16);
            for (int j=0; j<16; j++) {
              ImGui::TableNextColumn();
              if (i*16+j>=size) continue;
              if (depth == 8) {
                ImGui::Text("%.2x",regPool[i*16+j]);
              } else if (depth == 16) {
                ImGui::Text("%.4x",regPoolW[i*16+j]);
              } else {
                ImGui::Text("??");
              }
            }
          }
          ImGui::EndTable();
        }
        ImGui::PopFont();
      }
    }
  }
  if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)) curWindow=GUI_WINDOW_REGISTER_VIEW;
  ImGui::End();
}
