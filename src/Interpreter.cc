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
        if (!_memInstruction.validAddressingMode()) {
            return invalidOpcodeFault();
        }
        switch (_memInstruction.getOpcode()) {
            case Opcodes::balx:
                balx(getGPR(_memInstruction.getSrcDest()), computeAddress(_memInstruction));
                break;
            case Opcodes::bx:
                bx(computeAddress(_memInstruction));
                break;
            case Opcodes::callx:
                callx(computeAddress(_memInstruction));
                break;
            case Opcodes::stob:
                store(computeAddress(_memInstruction), getGPRValue<Ordinal>(_memInstruction.getSrcDest()), TreatAsByteOrdinal {});
                break;
            case Opcodes::stib:
                stib(static_cast<Integer>(getGPR(_memInstruction.getSrcDest())), computeAddress(_memInstruction));
                break;
            case Opcodes::stos:
                store(computeAddress(_memInstruction), getGPR(_memInstruction.getSrcDest()).getValue<Ordinal>(), TreatAs<ShortOrdinal>{});
                break;
            case Opcodes::stis:
                stis(static_cast<Integer>(getGPR(_memInstruction.getSrcDest())), computeAddress(_memInstruction));
                break;
            case Opcodes::st:
                store(computeAddress(_memInstruction), getGPR(_memInstruction.getSrcDest()).getValue<Ordinal>(), TreatAs<Ordinal>{});
                break;
            case Opcodes::stl:
                stl(_memInstruction, computeAddress(_memInstruction), getGPR(_memInstruction.getSrcDest(), TreatAsLongRegister{}));
                break;
            case Opcodes::stt:
                stt(_memInstruction, computeAddress(_memInstruction), getGPR(_memInstruction.getSrcDest(), TreatAsTripleRegister{}));
                break;
            case Opcodes::stq:
                stq(_memInstruction, computeAddress(_memInstruction), getGPR(_memInstruction.getSrcDest(), TreatAsQuadRegister{}));
                break;
            case Opcodes::ldob:
                getGPR(_memInstruction.getSrcDest()).setValue<Ordinal>(load(computeAddress(_memInstruction), TreatAs<ByteOrdinal>{}));
                break;
            case Opcodes::ldib:
                ldib(computeAddress(_memInstruction), getGPR(_memInstruction.getSrcDest()));
                break;
            case Opcodes::ldos:
                getGPR(_memInstruction.getSrcDest()).setValue<Ordinal>(load(computeAddress(_memInstruction), TreatAs<ShortOrdinal>{}));
                break;
            case Opcodes::ldis:
                ldis(computeAddress(_memInstruction), getGPR(_memInstruction.getSrcDest()));
                break;
            case Opcodes::ld:
                getGPR(_memInstruction.getSrcDest()).setValue<Ordinal>(load(computeAddress(_memInstruction), TreatAsOrdinal{}));
                break;
            case Opcodes::ldl:
                ldl(_memInstruction, computeAddress(_memInstruction), getGPR(_memInstruction.getSrcDest(), TreatAsLongRegister{}));
                break;
            case Opcodes::ldt:
                ldt(_memInstruction, computeAddress(_memInstruction), getGPR(_memInstruction.getSrcDest(), TreatAsTripleRegister{}));
                break;
            case Opcodes::ldq:
                ldq(_memInstruction, computeAddress(_memInstruction), getGPR(_memInstruction.getSrcDest(), TreatAsQuadRegister{}));
                break;
            case Opcodes::lda:
                setGPR(_memInstruction.getSrcDest(), computeAddress(_memInstruction), TreatAsOrdinal{});
                break;
            default:
                return unimplementedFault();
        }
        /// @todo temporary
        return std::nullopt;
    } else if (instruction_.isREGFormat()) {
        switch (_regInstruction.getOpcode()) {
            case Opcodes::notOperation:
                notOperation(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)));
                break;
            case Opcodes::andOperation:
                microcodedBitwiseOperation<AndOperation>(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::orOperation: // or
                microcodedBitwiseOperation<OrOperation>(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::xorOperation:
                microcodedBitwiseOperation<XorOperation>(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::notbit:
                notbit(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::andnot:
                andnot(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::setbit:
                setbit(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::notand: // notand
                notand(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::nor: // nor
                nor(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::xnor:
                xnor(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::ornot: // ornot
                ornot(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::nand: // nand
                nand(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::clrbit: // clrbit
                clrbit(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::notor: // notor
                notor(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::alterbit: // alterbit
                alterbit(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
                // in some of the opcodeExt values seem to reflect the resultant truth
                // table for the operation :). That's pretty cool
            case Opcodes::addo:
                addo(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::addi: // addi
                addi(getGPR(_regInstruction.getSrcDest()), static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::subo: // subo
                // I remember this trick from college, subtraction is just addition
                // with a negative second argument :). I never gave it much thought
                // until now but it seems to be an effective trick to save space.
                subo(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::subi: // subi
                subi(getGPR(_regInstruction.getSrcDest()), static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::shro: // shro
                shro(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::shrdi: // shrdi
                // according to the manual, equivalent to divi value, 2 so that is what we're going to do for correctness sake
                getGPR(_regInstruction.getSrcDest()).setValue<Integer>( static_cast<Integer>(getSrc1Register(_regInstruction)) < 32 && static_cast<Integer>(getSrc1Register(_regInstruction)) >= 0 ? static_cast<Integer>(getSrc2Register(_regInstruction)) / static_cast<Integer>(computeBitPosition(static_cast<Integer>(getSrc1Register(_regInstruction)))) : 0);
                break;
            case Opcodes::shri:
                shri(getGPR(_regInstruction.getSrcDest()), static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::shlo:
                shlo(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::rotate:
                rotate(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::shli:
                shli(getGPR(_regInstruction.getSrcDest()), static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::cmpo: // cmpo
                cmpo(static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::cmpi: // cmpi
                cmpi(static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::concmpo: // concmpo
                concmpo(static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::concmpi: // concmpi
                concmpi(static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::cmpinco: // cmpinco
                cmpinco(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::cmpinci: // cmpinci
                cmpinci(getGPR(_regInstruction.getSrcDest()), static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::cmpdeco: // cmpdeco
                cmpdeco(getGPR(_regInstruction.getSrcDest()), static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::cmpdeci: // cmpdeci
                cmpdeci(getGPR(_regInstruction.getSrcDest()), static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::scanbyte: // scanbyte
                scanbyte(static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::chkbit: // chkbit
                ac_.setConditionResult((static_cast<Ordinal>(getSrc2Register(_regInstruction)) & computeBitPosition(static_cast<Ordinal>(getSrc1Register(_regInstruction)))) == 0 );
                break;
            case Opcodes::addc:
                addc(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::subc:
                subc(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::mov:
                getGPR(_regInstruction.getSrcDest()).setValue<Ordinal>(static_cast<Ordinal>(getSrc1Register(_regInstruction)));
                break;
            case Opcodes::movl:
                movl(_regInstruction);
                //performRegisterTransfer(_regInstruction, 0b1, 2);
                break;
            case Opcodes::movt:
                movt(_regInstruction);
                //performRegisterTransfer(_regInstruction, 0b11, 3);
                break;
            case Opcodes::movq:
                movq(_regInstruction);
                //performRegisterTransfer(_regInstruction, 0b11, 4);
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
                mulo(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::muli:
                muli(getGPR(_regInstruction.getSrcDest()), static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::divi:
                divi(getGPR(_regInstruction.getSrcDest()), static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::divo:
                divo(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::remo:
                remo(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::remi:
                remi(getGPR(_regInstruction.getSrcDest()), static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::modi:
                modi(getGPR(_regInstruction.getSrcDest()), static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::modify:
                modify(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::extract:
                extract(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::modac:
                modxc(ac_, getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                // update the rounding mode
                updateRoundingMode();
                break;
            case Opcodes::modtc:
                modxc(tc_, getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::modpc:
                modpc(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::atadd:
                atadd(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::atmod:
                atmod(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::emul:
                emul(_regInstruction, getGPR(_regInstruction.getSrcDest(), TreatAsLongRegister{}), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::ediv:
                ediv(_regInstruction,
                     getGPR(_regInstruction.getSrcDest(), TreatAsLongRegister{}),
                     static_cast<Ordinal>(getSrc1Register(_regInstruction)),
                     getGPR(_regInstruction.getSrc2(), TreatAsLongRegister{}));
                break;
            case Opcodes::calls:
                return calls(static_cast<Ordinal>(getSrc1Register(_regInstruction)));
            case Opcodes::spanbit:
                spanbit(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)));
                break;
            case Opcodes::scanbit:
                scanbit(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)));
                break;
            case Opcodes::synld:
                synld(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)));
                break;
            case Opcodes::synmov:
                synmov(getSrc1Register(_regInstruction), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::synmovl:
                synmovl(getSrc1Register(_regInstruction), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::synmovq:
                synmovq(getSrc1Register(_regInstruction), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::selno:
            case Opcodes::sele:
            case Opcodes::selg:
            case Opcodes::selge:
            case Opcodes::sell:
            case Opcodes::selne:
            case Opcodes::selle:
            case Opcodes::selo:
                performSelect(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)));
                break;
            case Opcodes::addono:
            case Opcodes::addoe:
            case Opcodes::addog:
            case Opcodes::addoge:
            case Opcodes::addol:
            case Opcodes::addone:
            case Opcodes::addole:
            case Opcodes::addoo:
                performConditionalAdd(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)), TreatAsOrdinal{});
                break;

            case Opcodes::addino:
            case Opcodes::addie:
            case Opcodes::addig:
            case Opcodes::addige:
            case Opcodes::addil:
            case Opcodes::addine:
            case Opcodes::addile:
            case Opcodes::addio:
                performConditionalAdd(getGPR(_regInstruction.getSrcDest()), static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)), TreatAsInteger{});
                break;
            case Opcodes::subono:
            case Opcodes::suboe:
            case Opcodes::subog:
            case Opcodes::suboge:
            case Opcodes::subol:
            case Opcodes::subone:
            case Opcodes::subole:
            case Opcodes::suboo:
                performConditionalSubtract(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)), TreatAsOrdinal{});
                break;

            case Opcodes::subino:
            case Opcodes::subie:
            case Opcodes::subig:
            case Opcodes::subige:
            case Opcodes::subil:
            case Opcodes::subine:
            case Opcodes::subile:
            case Opcodes::subio:
                performConditionalSubtract(getGPR(_regInstruction.getSrcDest()), static_cast<Integer>(getSrc1Register(_regInstruction)), static_cast<Integer>(getSrc2Register(_regInstruction)), TreatAsInteger{});
                break;
            case Opcodes::dmovt:
                dmovt(getGPR(_regInstruction.getSrcDest()), static_cast<Ordinal>(getSrc1Register(_regInstruction)));
                break;
            case Opcodes::cmpstr:
                cmpstr(static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)), static_cast<Ordinal>(getGPR(_regInstruction.getSrcDest())));
                break;
            case Opcodes::movqstr:
                movqstr(static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)), static_cast<Ordinal>(getGPR(_regInstruction.getSrcDest())));
                break;
            case Opcodes::movstr:
                movstr(static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)), static_cast<Ordinal>(getGPR(_regInstruction.getSrcDest())));
                break;
            case Opcodes::fill:
                fill(static_cast<Ordinal>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(getSrc2Register(_regInstruction)), static_cast<Ordinal>(getGPR(_regInstruction.getSrcDest())));
                break;
            case Opcodes::ldphy:
                ldphy(static_cast<Address>(getSrc1Register(_regInstruction)), getGPR(_regInstruction.getSrcDest()));
                break;
#if 0
                case Opcodes::ldtime:
            ldtime(getGPR(_regInstruction.getSrcDest(), TreatAsLongRegister{}));
            break;
        case Opcodes::condwait:
            condwait(static_cast<Ordinal>(getSrc1Register(_regInstruction)));
            break;
        case Opcodes::inspacc:
            inspacc(static_cast<Ordinal>(getSrc1Register(_regInstruction)), getGPR(_regInstruction.getSrcDest()));
            break;
        case Opcodes::wait:
            wait(static_cast<SegmentSelector>(getSrc1Register(_regInstruction)));
            break;
        case Opcodes::condrec:
            condrec(static_cast<Address>(getSrc1Register(_regInstruction)), getGPR(_regInstruction.getSrcDest()) );
            break;
        case Opcodes::saveprcs:
            saveprcs();
            break;
        case Opcodes::resumprcs:
            resumprcs(static_cast<SegmentSelector>(getSrc1Register(_regInstruction)));
            break;
        case Opcodes::signal:
            signal(static_cast<SegmentSelector>(getSrc1Register(_regInstruction)));
            break;
        case Opcodes::send:
            send(static_cast<SegmentSelector>(getSrc1Register(_regInstruction)), static_cast<Ordinal>(src2), static_cast<SegmentSelector>(getGPR(_regInstruction.getSrcDest())));
            break;
        case Opcodes::sendserv:
            sendserv(static_cast<SegmentSelector>(getSrc1Register(_regInstruction)));
            break;
        case Opcodes::schedprcs:
            schedprcs(static_cast<SegmentSelector>(getSrc1Register(_regInstruction)));
            break;
        case Opcodes::receive:
            receive(static_cast<SegmentSelector>(getSrc1Register(_regInstruction)), getGPR(_regInstruction.getSrcDest()));
            break;
#endif

#define X(name) case Opcodes:: name : return name (_regInstruction)
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
Core::processInstruction(const COBRInstruction& cobr) {
    auto displacement = static_cast<ShortInteger>(cobr.getDisplacement());
    // there are two versions of COBR instructions
    // ones where the src1 field is a register and can act as a destination or source
    // the other where src1 is a literal and can only be used as a source
    // thus there are two paths to take here and not all versions are actually accessible in all forms
    if (cobr.getM1()) {
        switch(cobr.getOpcode()) {
            case Opcodes::bbc:
                bbc(cobr.getSrc1(), getSrc2Register(cobr), displacement);
                break;
            case Opcodes::bbs:
                bbs(cobr.getSrc1(), getSrc2Register(cobr), displacement);
                break;
            case Opcodes::cmpobg:
            case Opcodes::cmpobe:
            case Opcodes::cmpobge:
            case Opcodes::cmpobl:
            case Opcodes::cmpobne:
            case Opcodes::cmpoble:
                cmpobGeneric(cobr.getMask(), cobr.getSrc1(), static_cast<Ordinal>(getSrc2Register(cobr)), displacement);
                break;
            case Opcodes::cmpibno: // never branches
            case Opcodes::cmpibg:
            case Opcodes::cmpibe:
            case Opcodes::cmpibge:
            case Opcodes::cmpibl:
            case Opcodes::cmpibne:
            case Opcodes::cmpible:
            case Opcodes::cmpibo: // always branches
                cmpibGeneric(cobr.getMask(), cobr.getSrc1(), static_cast<Integer>(getSrc2Register(cobr)), displacement);
                break;
            default:
                // test instructions perform modifications to src1 so we must error out
                // in this case!
                return unimplementedFault();
        }
    } else {
        switch(cobr.getOpcode()) {
            case Opcodes::bbc:
                bbc(getSrc1Register(cobr), getSrc2Register(cobr), displacement);
                break;
            case Opcodes::bbs:
                bbs(getSrc1Register(cobr), getSrc2Register(cobr), displacement);
                break;
            case Opcodes::testno:
            case Opcodes::testg:
            case Opcodes::teste:
            case Opcodes::testge:
            case Opcodes::testl:
            case Opcodes::testne:
            case Opcodes::testle:
            case Opcodes::testo: {
                getSrc1Register(cobr).setValue<Ordinal>(fullConditionCodeCheck(cobr.getMask()) ? 1 : 0);
                break;
            }
            case Opcodes::cmpobg:
            case Opcodes::cmpobe:
            case Opcodes::cmpobge:
            case Opcodes::cmpobl:
            case Opcodes::cmpobne:
            case Opcodes::cmpoble:
                cmpobGeneric(cobr.getMask(),
                             static_cast<Ordinal>(getSrc1Register(cobr)),
                             static_cast<Ordinal>(getSrc2Register(cobr)),
                             displacement);
                break;
            case Opcodes::cmpibno: // never branches
            case Opcodes::cmpibg:
            case Opcodes::cmpibe:
            case Opcodes::cmpibge:
            case Opcodes::cmpibl:
            case Opcodes::cmpibne:
            case Opcodes::cmpible:
            case Opcodes::cmpibo: // always branches
                cmpibGeneric(cobr.getMask(),
                             static_cast<Integer>(getSrc1Register(cobr)),
                             static_cast<Integer>(getSrc2Register(cobr)),
                             displacement);
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
