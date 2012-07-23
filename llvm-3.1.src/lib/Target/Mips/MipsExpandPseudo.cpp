//===--  MipsExpandPseudo.cpp - Expand Pseudo Instructions ----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass expands pseudo instructions into target instructions after register
// allocation but before post-RA scheduling.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "mips-expand-pseudo"

#include "Mips.h"
#include "MipsTargetMachine.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/Statistic.h"

using namespace llvm;

namespace {
  struct MipsExpandPseudo : public MachineFunctionPass {

    TargetMachine &TM;
    const TargetInstrInfo *TII;

    static char ID;
    MipsExpandPseudo(TargetMachine &tm)
      : MachineFunctionPass(ID), TM(tm), TII(tm.getInstrInfo()) { }

    virtual const char *getPassName() const {
      return "Mips PseudoInstrs Expansion";
    }

    bool runOnMachineFunction(MachineFunction &F);
    bool runOnMachineBasicBlock(MachineBasicBlock &MBB);

  private:
    void ExpandBuildPairF64(MachineBasicBlock&, MachineBasicBlock::iterator);
    void ExpandExtractElementF64(MachineBasicBlock&,
                                 MachineBasicBlock::iterator);
  };
  char MipsExpandPseudo::ID = 0;
} // end of anonymous namespace

bool MipsExpandPseudo::runOnMachineFunction(MachineFunction& F) {
  bool Changed = false;

  for (MachineFunction::iterator I = F.begin(); I != F.end(); ++I)
    Changed |= runOnMachineBasicBlock(*I);

  return Changed;
}

bool MipsExpandPseudo::runOnMachineBasicBlock(MachineBasicBlock& MBB) {

  bool Changed = false;
  MachineBasicBlock::iterator I0, I1;
  for (MachineBasicBlock::iterator I = MBB.begin(); I != MBB.end();) {
    const MCInstrDesc& MCid = I->getDesc();

    switch(MCid.getOpcode()) {
    default:
      ++I;
      continue;
    case Mips::SETGP2:
      // Convert "setgp2 $globalreg, $t9" to "addu $globalreg, $v0, $t9"
      BuildMI(MBB, I, I->getDebugLoc(), TII->get(Mips::ADDu),
              I->getOperand(0).getReg())
        .addReg(Mips::V0).addReg(I->getOperand(1).getReg());
      break;

	// trick for SH
	case Mips::ULW:
	  I0 = I++;
	  I1 = I++;
	  BuildMI(MBB, I0, I0->getDebugLoc(), TII->get(Mips::SB)).addOperand(I1->getOperand(1))
			  .addOperand(I0->getOperand(1)).addOperand(I0->getOperand(2));
	  BuildMI(MBB, I0, I0->getDebugLoc(), TII->get(Mips::SRL)).addOperand(I0->getOperand(0))
			  .addOperand(I1->getOperand(1)).addImm(8);
	  BuildMI(MBB, I0, I0->getDebugLoc(), TII->get(Mips::SB)).addOperand(I1->getOperand(1))
			  .addOperand(I0->getOperand(1)).addImm(I0->getOperand(2).getImm() + 1);

	  MBB.erase(I0);
	  MBB.erase(I1);
	  MBB.erase(I++);
	  
	  continue;

	// trick for LH
	case Mips::ULH:
	  I->dump();
	  BuildMI(MBB, I, I->getDebugLoc(), TII->get(Mips::LB)).addOperand(I->getOperand(0)).
			  addOperand(I->getOperand(1)).addImm(I->getOperand(2).getImm()+1);
	  break;

	// trick for LHu
	case Mips::ULHu:
	  I->dump();
	  BuildMI(MBB, I, I->getDebugLoc(), TII->get(Mips::LBu)).addOperand(I->getOperand(0)).
			  addOperand(I->getOperand(1)).addImm(I->getOperand(2).getImm()+1);
	  break;

	case Mips::SYNC:
	  I->dump();
	  BuildMI(MBB, I, I->getDebugLoc(), TII->get(Mips::NOP));
	  break;

	case Mips::ADD:
	  I->dump();
	  BuildMI(MBB, I, I->getDebugLoc(), TII->get(Mips::ADD)).addOperand(I->getOperand(0)).
			  addOperand(I->getOperand(1)).addOperand(I->getOperand(2));
	  break;

	case Mips::ADDi:
	  I->dump();
	  BuildMI(MBB, I, I->getDebugLoc(), TII->get(Mips::ADDi)).addOperand(I->getOperand(0)).
			  addOperand(I->getOperand(1)).addOperand(I->getOperand(2));
	  break;

	case Mips::SUB:
	  I->dump();
	  BuildMI(MBB, I, I->getDebugLoc(), TII->get(Mips::SUBu)).addOperand(I->getOperand(0)).
			  addOperand(I->getOperand(1)).addOperand(I->getOperand(2));
	  break;

	case Mips::B:
	  I->dump();
	  BuildMI(MBB, I, I->getDebugLoc(), TII->get(Mips::BGEZ)).addReg(Mips::ZERO).
			  addOperand(I->getOperand(0));
	  break;


	case Mips::LWL:
	case Mips::LWR:
	case Mips::SWL:
	case Mips::SWR:

	case Mips::LL:
	case Mips::LL_P8:
	case Mips::SC:
	case Mips::SC_P8:

	case Mips::MOVZ_I_I:

	case Mips::ROTR:
	case Mips::ROTRV:

	case Mips::EXT:
	case Mips::INS:
	case Mips::RDHWR:

	case Mips::MUL:
	case Mips::MULTu:
	case Mips::MULT:
	case Mips::SDIV:
	case Mips::UDIV:

	/*
	case Mips::MTHI:
	case Mips::MTLO:
	case Mips::MFHI:
	case Mips::MFLO:
	*/

	
	/*
	case Mips::BGEZAL:
	case Mips::BLTZAL:
	*/


	  I->dump();
	  ++I;
	  continue;
	
	
    case Mips::BuildPairF64:
      ExpandBuildPairF64(MBB, I);
      break;
    case Mips::ExtractElementF64:
      ExpandExtractElementF64(MBB, I);
      break;
    }

    // delete original instr
    MBB.erase(I++);
    Changed = true;
  }

  return Changed;
}

void MipsExpandPseudo::ExpandBuildPairF64(MachineBasicBlock& MBB,
                                            MachineBasicBlock::iterator I) {
  unsigned DstReg = I->getOperand(0).getReg();
  unsigned LoReg = I->getOperand(1).getReg(), HiReg = I->getOperand(2).getReg();
  const MCInstrDesc& Mtc1Tdd = TII->get(Mips::MTC1);
  DebugLoc dl = I->getDebugLoc();
  const uint16_t* SubReg =
    TM.getRegisterInfo()->getSubRegisters(DstReg);

  // mtc1 Lo, $fp
  // mtc1 Hi, $fp + 1
  BuildMI(MBB, I, dl, Mtc1Tdd, *SubReg).addReg(LoReg);
  BuildMI(MBB, I, dl, Mtc1Tdd, *(SubReg + 1)).addReg(HiReg);
}

void MipsExpandPseudo::ExpandExtractElementF64(MachineBasicBlock& MBB,
                                               MachineBasicBlock::iterator I) {
  unsigned DstReg = I->getOperand(0).getReg();
  unsigned SrcReg = I->getOperand(1).getReg();
  unsigned N = I->getOperand(2).getImm();
  const MCInstrDesc& Mfc1Tdd = TII->get(Mips::MFC1);
  DebugLoc dl = I->getDebugLoc();
  const uint16_t* SubReg = TM.getRegisterInfo()->getSubRegisters(SrcReg);

  BuildMI(MBB, I, dl, Mfc1Tdd, DstReg).addReg(*(SubReg + N));
}

/// createMipsMipsExpandPseudoPass - Returns a pass that expands pseudo
/// instrs into real instrs
FunctionPass *llvm::createMipsExpandPseudoPass(MipsTargetMachine &tm) {
  return new MipsExpandPseudo(tm);
}
