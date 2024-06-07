// sim5
// Copyright (c) 2022-2023, Joshua Scoggins
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "Types.h"
#include "Core.h"
#include "BinaryOperations.h"
#include "Disassembly.h"
#include <iostream>

OptionalFaultRecord
Core::doDispatchInternal() noexcept {
    if (instruction_.isCTRL()) {
        return processInstruction(CTRLInstruction{instruction_});
    } else if (instruction_.isCOBR()) {
        return processInstruction(COBRInstruction{instruction_});
    } else if (instruction_.isMEMFormat()) {
        // always load the next word for simplicity
        return processInstruction(MEMInstruction{instruction_, load(ip_.o + 4, TreatAsInteger{})});
    } else if (instruction_.isREGFormat()) {
        return processInstruction(REGInstruction{instruction_});
    } else {
        return unimplementedFault();
    }
}
void
Core::cycle() noexcept {
    instruction_.setValue(load(ip_.a, TreatAsOrdinal{}), TreatAsOrdinal{});
    instructionLength_ = 4;
    advanceInstruction_ = true;
    if (auto result = doDispatchInternal(); result) {
        generateFault(*result);
    }
    if (advanceInstruction_) {
        nextInstruction();
    }
    // according to the blue book they use a counter modulo 8 as a pseudo-random source
    // let's add it in ours as well for whatever we want!
    ++pseudoRandomSource_;
}

OptionalFaultRecord
Core::processInstruction(const REGInstruction & inst) {
    auto& regDest = getGPR(inst.getSrcDest());
    const auto& src1 = getSrc1Register(inst);
    const auto& src2 = getSrc2Register(inst);
    switch (inst.getOpcode()) {
        case Opcodes::notOperation:
            notOperation(regDest, static_cast<Ordinal>(src1));
            break;
        case Opcodes::andOperation:
            microcodedBitwiseOperation<AndOperation>(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::orOperation: // or
            microcodedBitwiseOperation<OrOperation>(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::xorOperation:
            microcodedBitwiseOperation<XorOperation>(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::notbit:
            notbit(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::andnot:
            andnot(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::setbit:
            setbit(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::notand: // notand
            notand(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::nor: // nor
            nor(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::xnor:
            xnor(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::ornot: // ornot
            ornot(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::nand: // nand
            nand(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::clrbit: // clrbit
            clrbit(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::notor: // notor
            notor(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::alterbit: // alterbit
            alterbit(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
            // in some of the opcodeExt values seem to reflect the resultant truth
            // table for the operation :). That's pretty cool
        case Opcodes::addo:
            addo(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::addi: // addi
            addi(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::subo: // subo
            // I remember this trick from college, subtraction is just addition
            // with a negative second argument :). I never gave it much thought
            // until now but it seems to be an effective trick to save space.
            subo(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::subi: // subi
            subi(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::shro: // shro
            shro(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::shrdi: // shrdi
            // according to the manual, equivalent to divi value, 2 so that is what we're going to do for correctness sake
            regDest.setValue<Integer>( static_cast<Integer>(src1) < 32 && static_cast<Integer>(src1) >= 0 ? static_cast<Integer>(src2) / static_cast<Integer>(computeBitPosition(static_cast<Integer>(src1))) : 0);
            break;
        case Opcodes::shri:
            shri(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::shlo:
            shlo(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::rotate:
            rotate(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::shli:
            shli(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::cmpo: // cmpo
            cmpo(static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::cmpi: // cmpi
            cmpi(static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::concmpo: // concmpo
            concmpo(static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::concmpi: // concmpi
            concmpi(static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::cmpinco: // cmpinco
            cmpinco(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::cmpinci: // cmpinci
            cmpinci(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::cmpdeco: // cmpdeco
            cmpdeco(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::cmpdeci: // cmpdeci
            cmpdeci(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::scanbyte: // scanbyte
            scanbyte(static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::chkbit: // chkbit
            ac_.setConditionResult((static_cast<Ordinal>(src2) & computeBitPosition(static_cast<Ordinal>(src1))) == 0 );
            break;
        case Opcodes::addc:
            addc(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::subc:
            subc(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::mov:
            regDest.setValue<Ordinal>(static_cast<Ordinal>(src1));
            break;
        case Opcodes::movl:
            movl(inst);
            //performRegisterTransfer(inst, 0b1, 2);
            break;
        case Opcodes::movt:
            movt(inst);
            //performRegisterTransfer(inst, 0b11, 3);
            break;
        case Opcodes::movq:
            movq(inst);
            //performRegisterTransfer(inst, 0b11, 4);
            break;
        case Opcodes::syncf:
            syncf();
            break;
        case Opcodes::flushreg:
            flushreg();
            break;
        case Opcodes::fmark:
            fmark();
            break;
        case Opcodes::mark:
            mark();
            break;
        case Opcodes::mulo:
            mulo(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::muli:
            muli(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::divi:
            divi(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::divo:
            divo(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::remo:
            remo(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::remi:
            remi(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::modi:
            modi(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::modify:
            modify(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::extract:
            extract(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::modac:
            modxc(ac_, regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            // update the rounding mode
            updateRoundingMode();
            break;
        case Opcodes::modtc:
            modxc(tc_, regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::modpc:
            modpc(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::atadd:
            atadd(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::atmod:
            atmod(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::emul:
            emul(inst, getGPR(inst.getSrcDest(), TreatAsLongRegister{}), static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::ediv:
            ediv(inst,
                 getGPR(inst.getSrcDest(), TreatAsLongRegister{}),
                 static_cast<Ordinal>(src1),
                 getGPR(inst.getSrc2(), TreatAsLongRegister{}));
            break;
        case Opcodes::calls:
            return calls(static_cast<Ordinal>(src1));
        case Opcodes::spanbit:
            spanbit(regDest, static_cast<Ordinal>(src1));
            break;
        case Opcodes::scanbit:
            scanbit(regDest, static_cast<Ordinal>(src1));
            break;
        case Opcodes::synld:
            synld(regDest, static_cast<Ordinal>(src1));
            break;
        case Opcodes::synmov:
            synmov(src1, static_cast<Ordinal>(src2));
            break;
        case Opcodes::synmovl:
            synmovl(src1, static_cast<Ordinal>(src2));
            break;
        case Opcodes::synmovq:
            synmovq(src1, static_cast<Ordinal>(src2));
            break;
        case Opcodes::selno:
        case Opcodes::sele:
        case Opcodes::selg:
        case Opcodes::selge:
        case Opcodes::sell:
        case Opcodes::selne:
        case Opcodes::selle:
        case Opcodes::selo:
            performSelect(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::addono:
        case Opcodes::addoe:
        case Opcodes::addog:
        case Opcodes::addoge:
        case Opcodes::addol:
        case Opcodes::addone:
        case Opcodes::addole:
        case Opcodes::addoo:
            performConditionalAdd(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2), TreatAsOrdinal{});
            break;

        case Opcodes::addino:
        case Opcodes::addie:
        case Opcodes::addig:
        case Opcodes::addige:
        case Opcodes::addil:
        case Opcodes::addine:
        case Opcodes::addile:
        case Opcodes::addio:
            performConditionalAdd(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2), TreatAsInteger{});
            break;
        case Opcodes::subono:
        case Opcodes::suboe:
        case Opcodes::subog:
        case Opcodes::suboge:
        case Opcodes::subol:
        case Opcodes::subone:
        case Opcodes::subole:
        case Opcodes::suboo:
            performConditionalSubtract(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2), TreatAsOrdinal{});
            break;

        case Opcodes::subino:
        case Opcodes::subie:
        case Opcodes::subig:
        case Opcodes::subige:
        case Opcodes::subil:
        case Opcodes::subine:
        case Opcodes::subile:
        case Opcodes::subio:
            performConditionalSubtract(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2), TreatAsInteger{});
            break;
        case Opcodes::dmovt:
            dmovt(regDest, static_cast<Ordinal>(src1));
            break;
        case Opcodes::cmpstr:
            cmpstr(static_cast<Ordinal>(src1), static_cast<Ordinal>(src2), static_cast<Ordinal>(regDest));
            break;
        case Opcodes::movqstr:
            movqstr(static_cast<Ordinal>(src1), static_cast<Ordinal>(src2), static_cast<Ordinal>(regDest));
            break;
        case Opcodes::movstr:
            movstr(static_cast<Ordinal>(src1), static_cast<Ordinal>(src2), static_cast<Ordinal>(regDest));
            break;
        case Opcodes::fill:
            fill(static_cast<Ordinal>(src1), static_cast<Ordinal>(src2), static_cast<Ordinal>(regDest));
            break;
        case Opcodes::ldphy:
            ldphy(static_cast<Address>(src1), regDest);
            break;
#if 0
        case Opcodes::ldtime:
            ldtime(getGPR(inst.getSrcDest(), TreatAsLongRegister{}));
            break;
        case Opcodes::condwait:
            condwait(static_cast<Ordinal>(src1));
            break;
        case Opcodes::inspacc:
            inspacc(static_cast<Ordinal>(src1), regDest);
            break;
        case Opcodes::wait:
            wait(static_cast<SegmentSelector>(src1));
            break;
        case Opcodes::condrec:
            condrec(static_cast<Address>(src1), regDest );
            break;
        case Opcodes::saveprcs:
            saveprcs();
            break;
        case Opcodes::resumprcs:
            resumprcs(static_cast<SegmentSelector>(src1));
            break;
        case Opcodes::signal:
            signal(static_cast<SegmentSelector>(src1));
            break;
        case Opcodes::send:
            send(static_cast<SegmentSelector>(src1), static_cast<Ordinal>(src2), static_cast<SegmentSelector>(regDest));
            break;
        case Opcodes::sendserv:
            sendserv(static_cast<SegmentSelector>(src1));
            break;
        case Opcodes::schedprcs:
            schedprcs(static_cast<SegmentSelector>(src1));
            break;
        case Opcodes::receive:
            receive(static_cast<SegmentSelector>(src1), regDest);
            break;
#endif

#define X(name) case Opcodes:: name : return name (inst)
        X(addr);
        X(addrl);
        X(atanr);
        X(atanrl);
        X(classr);
        X(classrl);
        X(cmpr);
        X(cmprl);
            //X(cmpor);
            //X(cmporl);
        X(cosr);
        X(cosrl);
        X(cpyrsre);
        X(cpysre);
            //X(cvtri);
            //X(cvtril);
        X(cvtzri);
            //X(cvtzril);
            //X(cvtilr);
        X(cvtir);
        X(divr);
        X(divrl);
            //X(movr);
        X(movre);
        X(movr);
        X(movrl);
        X(mulr);
        X(mulrl);
            //X(remr);
            //X(remrl);
        X(roundr);
        X(roundrl);
        X(subr);
        X(subrl);
            //X(scaler);
            //X(scalerl);
        X(sinr);
        X(sinrl);
        X(sqrtr);
        X(sqrtrl);
        X(tanr);
        X(tanrl);
            //X(logeprl);
            //X(logepr);
            //X(logbnrl);
            //X(logbnr);
            //X(logr);
            //X(logrl);

#undef X
        default:
            return unimplementedFault();
        //default:
        //    return processFPInstruction(inst);
    }
    /// @todo temporary
    return std::nullopt;
}
OptionalFaultRecord
Core::processInstruction(const MEMInstruction & inst) {
    if (auto eao = computeAddress(inst); eao) {
        Register &srcDest= getGPR(inst.getSrcDest());
        auto effectiveAddress = *eao;
        switch (inst.getOpcode()) {
            case Opcodes::balx:
                balx(srcDest, effectiveAddress);
                break;
            case Opcodes::bx:
                bx(effectiveAddress);
                break;
            case Opcodes::callx:
                callx(effectiveAddress);
                break;
            case Opcodes::stob:
                store(effectiveAddress, srcDest.getValue<Ordinal>(), TreatAs<ByteOrdinal>{});
                break;
            case Opcodes::stib:
                stib(static_cast<Integer>(srcDest), effectiveAddress);
                break;
            case Opcodes::stos:
                store(effectiveAddress, srcDest.getValue<Ordinal>(), TreatAs<ShortOrdinal>{});
                break;
            case Opcodes::stis:
                stis(static_cast<Integer>(srcDest), effectiveAddress);
                break;
            case Opcodes::st:
                store(effectiveAddress, srcDest.getValue<Ordinal>(), TreatAs<Ordinal>{});
                break;
            case Opcodes::stl:
                stl(inst, effectiveAddress, getGPR(inst.getSrcDest(), TreatAsLongRegister{}));
                break;
            case Opcodes::stt:
                stt(inst, effectiveAddress, getGPR(inst.getSrcDest(), TreatAsTripleRegister{}));
                break;
            case Opcodes::stq:
                stq(inst, effectiveAddress, getGPR(inst.getSrcDest(), TreatAsQuadRegister{}));
                break;
            case Opcodes::ldob:
                srcDest.setValue<Ordinal>(load(effectiveAddress, TreatAs<ByteOrdinal>{}));
                break;
            case Opcodes::ldib:
                ldib(effectiveAddress, srcDest);
                break;
            case Opcodes::ldos:
                srcDest.setValue<Ordinal>(load(effectiveAddress, TreatAs<ShortOrdinal>{}));
                break;
            case Opcodes::ldis:
                ldis(effectiveAddress, srcDest);
                break;
            case Opcodes::ld:
                srcDest.setValue<Ordinal>(load(effectiveAddress, TreatAsOrdinal{}));
                break;
            case Opcodes::ldl:
                ldl(inst, effectiveAddress, getGPR(inst.getSrcDest(), TreatAsLongRegister{}));
                break;
            case Opcodes::ldt:
                ldt(inst, effectiveAddress, getGPR(inst.getSrcDest(), TreatAsTripleRegister{}));
                break;
            case Opcodes::ldq:
                ldq(inst, effectiveAddress, getGPR(inst.getSrcDest(), TreatAsQuadRegister{}));
                break;
            case Opcodes::lda:
                srcDest.setValue<Ordinal>(effectiveAddress);
                break;
            default:
                return unimplementedFault();
        }
    } else {
        return invalidOpcodeFault();
    }
    /// @todo temporary
    return std::nullopt;
}

OptionalFaultRecord
Core::processInstruction(const COBRInstruction& cobr) {
    auto opcode = cobr.getOpcode();
    auto displacement = static_cast<ShortInteger>(cobr.getDisplacement());
    auto mask = cobr.getMask();
    // there are two versions of COBR instructions
    // ones where the src1 field is a register and can act as a destination or source
    // the other where src1 is a literal and can only be used as a source
    // thus there are two paths to take here and not all versions are actually accessible in all forms
    if (auto& src2 = getSrc2Register(cobr); cobr.getM1()) {
        auto src1 = static_cast<uint8_t>(cobr.getSrc1());
        switch(opcode) {
            case Opcodes::bbc:
                bbc(src1, src2, displacement);
                break;
            case Opcodes::bbs:
                bbs(src1, src2, displacement);
                break;
            case Opcodes::cmpobg:
            case Opcodes::cmpobe:
            case Opcodes::cmpobge:
            case Opcodes::cmpobl:
            case Opcodes::cmpobne:
            case Opcodes::cmpoble:
                cmpobGeneric(mask, src1, static_cast<Ordinal>(src2), displacement);
                break;
            case Opcodes::cmpibno: // never branches
            case Opcodes::cmpibg:
            case Opcodes::cmpibe:
            case Opcodes::cmpibge:
            case Opcodes::cmpibl:
            case Opcodes::cmpibne:
            case Opcodes::cmpible:
            case Opcodes::cmpibo: // always branches
                cmpibGeneric(mask, src1, static_cast<Integer>(src2), displacement);
                break;
            default:
                // test instructions perform modifications to src1 so we must error out
                // in this case!
                return unimplementedFault();
        }
    } else {
        auto& src1 = getSrc1Register(cobr);
        switch(opcode) {
            case Opcodes::bbc:
                bbc(src1, src2, displacement);
                break;
            case Opcodes::bbs:
                bbs(src1, src2, displacement);
                break;
            case Opcodes::testno:
            case Opcodes::testg:
            case Opcodes::teste:
            case Opcodes::testge:
            case Opcodes::testl:
            case Opcodes::testne:
            case Opcodes::testle:
            case Opcodes::testo: {
                src1.setValue<Ordinal>(fullConditionCodeCheck(mask) ? 1 : 0);
                break;
            }
            case Opcodes::cmpobg:
            case Opcodes::cmpobe:
            case Opcodes::cmpobge:
            case Opcodes::cmpobl:
            case Opcodes::cmpobne:
            case Opcodes::cmpoble:
                cmpobGeneric(mask, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2), displacement);
                break;
            case Opcodes::cmpibno: // never branches
            case Opcodes::cmpibg:
            case Opcodes::cmpibe:
            case Opcodes::cmpibge:
            case Opcodes::cmpibl:
            case Opcodes::cmpibne:
            case Opcodes::cmpible:
            case Opcodes::cmpibo: // always branches
                cmpibGeneric(mask, static_cast<Integer>(src1), static_cast<Integer>(src2), displacement);
                break;
            default:
                return unimplementedFault();
        }
    }
    return std::nullopt;
}

OptionalFaultRecord
Core::processInstruction(const CTRLInstruction &instruction) {
    switch (instruction.getOpcode()) {
        case Opcodes::bal: // bal
            bal(instruction.getDisplacement());
            break;
        case Opcodes::b: // b
            branch(instruction.getDisplacement());
            break;
        case Opcodes::call: // call
            call(instruction.getDisplacement());
            break;
        case Opcodes::ret: // ret
            return ret();
        case Opcodes::bno:
        case Opcodes::be:
        case Opcodes::bne:
        case Opcodes::bl:
        case Opcodes::ble:
        case Opcodes::bg:
        case Opcodes::bge:
        case Opcodes::bo:
            // the branch instructions have the mask encoded into the opcode
            // itself so we can just use it and save a ton of space overall
            branchConditional(fullConditionCodeCheck(), instruction.getDisplacement());
            break;
        case Opcodes::faultno:
        case Opcodes::faulte:
        case Opcodes::faultne:
        case Opcodes::faultl:
        case Opcodes::faultle:
        case Opcodes::faultg:
        case Opcodes::faultge:
        case Opcodes::faulto:
            return faultGeneric();
        default:
            return unimplementedFault();
    }
    return std::nullopt;
}
