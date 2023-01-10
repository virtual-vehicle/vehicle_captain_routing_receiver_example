//
// Created by christophpilz on 14.09.2021.
//
// Authors:
// - Christoph Pilz (christoph.pilz@v2c2.at)
//
// reference: https://bytefreaks.net/programming-2/c/asn1c-full-working-example-of-asn-1-in-cc
//
// List of possible improvements
// [] Implement full parsing of DENM
// [] Implement full parsing of CAM
// [] Implement full parsing of POI
// [] Implement full parsing of SPATEM
// [] Implement full parsing of MAPEM
// [] Implement full parsing of IVIM
// [] Implement full parsing of EV_RSR
// [] Implement full parsing of TISTPGTRANSACTION
// [] Implement full parsing of SREM
// [] Implement full parsing of SSEM
// [] Implement full parsing of EVCSN
// [] Implement full parsing of SAEM
// [] Implement full parsing of RTCMEM
//

#include <chrono>
#include <iostream>
#include <thread>
#include <string>

#include <csignal>

#include "zmq.hpp"

extern "C" {
#include "vcits/denm/DENM.h"
#include "vcits/cam/CAM.h"
#include "vcits/cam/LowFrequencyContainer.h"       // needed, as it is not in the CAM.h include chain (only forward declarations)
#include "vcits/cam/SpecialVehicleContainer.h"     // needed, as it is not in the CAM.h include chain (only forward declarations)
#include "vcits/spatem/SPATEM.h"
#include "vcits/mapem/MAPEM.h"
#include "vcits/ivim/IVIM.h"
#include "vcits/srem/SREM.h"
#include "vcits/ssem/SSEM.h"
#include "vcits/rtcmem/RTCMEM.h"
#include "vcits/parser/Decoder.h"
}


// have means of clear exit on Ctrl+C
sig_atomic_t stopFlag = 0;

void signalHandler(int) {
    stopFlag = 1;
}

// prototypes
long get_message_id(void *msg, size_t size);
void print_denm(void *msg, size_t size);
void print_cam(void *msg, size_t size);
void print_poi(void *msg, size_t size);
void print_spatem(void *msg, size_t size);
void print_mapem(void *msg, size_t size);
void print_ivim(void *msg, size_t size);
void print_ev_rsr(void *msg, size_t size);
void print_tistpgtransaction(void *msg, size_t size);
void print_srem(void *msg, size_t size);
void print_ssem(void *msg, size_t size);
void print_evcsn(void *msg, size_t size);
void print_saem(void *msg, size_t size);
void print_rtcmem(void *msg, size_t size);

// main
int main(int argc, char* argv[]) {
    std::cout << "main()" << std::endl;
    // have means of clear exit on Ctrl+Csj6
    signal(SIGINT, &signalHandler);

    // variables
    int ret = 0;
    long v2x_message_id = 0;
    zmq::context_t zmq_context(1);
    zmq::socket_t zmq_socket_in(zmq_context, ZMQ_SUB);
    zmq::message_t topic;
    zmq::message_t msg;
    std::string zmq_connect_string;

    if(argc > 1) {
        zmq_connect_string.append("tcp://");
        zmq_connect_string.append(argv[1]);
        zmq_connect_string.append(":");
        zmq_connect_string.append(argv[2]);
    } else {
        std::cout << "Usage: " << argv[0] << " ip port" << std::endl;
        return 0;
    }

    // connect to ZMQ
    std::cout << "main(): " << "connect ZMQ (" << zmq_connect_string << ")" << std::endl;
    zmq_socket_in.connect(zmq_connect_string);
    zmq_socket_in.set(zmq::sockopt::subscribe, "");

    std::cout << "main(): " << "main loop" << std::endl;
    while(!stopFlag) {
        // receive topic
        if(zmq_socket_in.recv(topic, zmq::recv_flags::dontwait)) {
            // receive content
            if (zmq_socket_in.recv(msg, zmq::recv_flags::dontwait)) {
                // check message type
                std::string topic_string = std::string(static_cast<char *>(topic.data()), topic.size());
                if(topic_string.rfind("v2x.", 0) == 0) {
                    // we got a v2x message
                } else if(topic_string.rfind("raw.", 0) == 0){
                    // we got a raw message
                    continue; // skip message
                } else if(topic_string.rfind("hb.", 0) == 0){
                    // we got a heartbeat message
                    continue; // skip message
                } else {
                    // something is wrong?
                    std::cout << "ASYNC";
                }

                // print v2x_msg
                std::cout << "*** *** V2X Message Received *** ***" << std::endl;
                v2x_message_id = get_message_id(msg.data(), msg.size());
                switch (v2x_message_id) {
                    case ItsPduHeader__messageID_denm:
                        print_denm(msg.data(), msg.size());
                        break;
                    case ItsPduHeader__messageID_cam:
                        print_cam(msg.data(), msg.size());
                        break;
                    case ItsPduHeader__messageID_poi:
                        print_poi(msg.data(), msg.size());
                        break;
                    case ItsPduHeader__messageID_spatem:
                        print_spatem(msg.data(), msg.size());
                        break;
                    case ItsPduHeader__messageID_mapem:
                        print_mapem(msg.data(), msg.size());
                        break;
                    case ItsPduHeader__messageID_ivim:
                        print_ivim(msg.data(), msg.size());
                        break;
                    case ItsPduHeader__messageID_ev_rsr:
                        print_ev_rsr(msg.data(), msg.size());
                        break;
                    case ItsPduHeader__messageID_tistpgtransaction:
                        print_tistpgtransaction(msg.data(), msg.size());
                        break;
                    case ItsPduHeader__messageID_srem:
                        print_srem(msg.data(), msg.size());
                        break;
                    case ItsPduHeader__messageID_ssem:
                        print_ssem(msg.data(), msg.size());
                        break;
                    case ItsPduHeader__messageID_evcsn:
                        print_evcsn(msg.data(), msg.size());
                        break;
                    case ItsPduHeader__messageID_saem:
                        print_saem(msg.data(), msg.size());
                        break;
                    case ItsPduHeader__messageID_rtcmem:
                        print_rtcmem(msg.data(), msg.size());
                        break;
                    default:
                        std::cout << "Message not supported by this demo" << std::endl;
                        break;
                }
            }
        }
        std::this_thread::yield();
    }

    std::cout << "done" << std::endl;
    return ret;
}

// analysis functions
long get_message_id(void *msg, size_t size){
    long msg_id = 0;
    ItsPduHeader_t *its_pdu_header = nullptr;

    its_pdu_header = (ItsPduHeader_t *) Decoder::decode(&asn_DEF_ItsPduHeader, msg, size);

    std::cout << "ItsPduHeader" << std::endl;
    std::cout << "|-Protocol Version: " << its_pdu_header->protocolVersion << std::endl;
    std::cout << "|-Message ID: " << its_pdu_header->messageID << std::endl;
    std::cout << "|-Station ID: " << its_pdu_header->stationID << std::endl;

    msg_id = its_pdu_header->messageID;

    ASN_STRUCT_FREE(asn_DEF_ItsPduHeader, its_pdu_header);
    its_pdu_header = nullptr;
    return msg_id;
}

// printing functions
void print_denm(void *msg, size_t size){
    std::cout << "--- --- --- DENM --- --- ---" << std::endl;
    if( msg && size)
        std::cout << "print_denm(): " << "not yet implemented" << std::endl;
}
void print_cam(void *msg, size_t size) {
    std::cout << "--- --- --- CAM --- --- ---" << std::endl;

    CAM_t *cam = nullptr;

    cam = (CAM_t *) Decoder::decode(&asn_DEF_CAM, msg, size);

    if(cam->header.protocolVersion != 2){
        std::cout << "this sample code only supports version 2 encoding" << std::endl;
        return;
    }

    std::cout << "CAM" << std::endl;
    std::cout << "|-header:" << std::endl;
    std::cout << " |-protocolVersion: " << cam->header.protocolVersion << std::endl;
    std::cout << " |-messageID: " << cam->header.messageID << std::endl;
    std::cout << " |-stationID: " << cam->header.stationID << std::endl;
    std::cout << "|-cam:" << std::endl;
    std::cout << " |-generationDeltaTime: " << cam->cam.generationDeltaTime << std::endl;
    std::cout << " |-camParameters: " << std::endl;
    std::cout << "  |-basicContainer: " << std::endl;
    std::cout << "   |-stationType: " << cam->cam.camParameters.basicContainer.stationType << std::endl;
    std::cout << "   |-referencePosition: " << std::endl;
    std::cout << "    |-latitude: " << cam->cam.camParameters.basicContainer.referencePosition.latitude << std::endl;
    std::cout << "    |-longitude: " << cam->cam.camParameters.basicContainer.referencePosition.longitude << std::endl;
    std::cout << "    |-positionConfidenceEllipse: " << std::endl;
    std::cout << "     |-semiMajorConfidence: " << cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence << std::endl;
    std::cout << "     |-semiMinorConfidence: " << cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence << std::endl;
    std::cout << "     |-semiMajorOrientation: " << cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation << std::endl;
    std::cout << "    |-altitude: " << std::endl;
    std::cout << "     |-altitudeValue: " << cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue << std::endl;
    std::cout << "     |-altitudeConfidence: " << cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence << std::endl;
    std::cout << "  |-highFrequencyContainer: " << std::endl;
    std::cout << "   |-present: " << cam->cam.camParameters.highFrequencyContainer.present << std::endl;
    switch (cam->cam.camParameters.highFrequencyContainer.present) {
        case(HighFrequencyContainer_PR_NOTHING):
            std::cout << "   |-choice: " << "HighFrequencyContainer_PR_NOTHING" << std::endl;
            break;
        case(HighFrequencyContainer_PR_basicVehicleContainerHighFrequency):
            std::cout << "   |-choice: " << "HighFrequencyContainer_PR_basicVehicleContainerHighFrequency" << std::endl;
            std::cout << "    |-basicVehicleContainerHighFrequency: " << std::endl;
            std::cout << "     |-heading: " << std::endl;
            std::cout << "      |-headingValue: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue << std::endl;
            std::cout << "      |-headingConfidence: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence << std::endl;
            std::cout << "     |-speed: " << std::endl;
            std::cout << "      |-speedValue: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue << std::endl;
            std::cout << "      |-speedConfidence: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedConfidence << std::endl;
            std::cout << "     |-driveDirection: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection << std::endl;
            std::cout << "     |-vehicleLength: " << std::endl;
            std::cout << "      |-vehicleLengthValue: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue << std::endl;
            std::cout << "      |-vehicleLengthConfidenceIndication: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication << std::endl;
            std::cout << "     |-vehicleWidth: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth << std::endl;
            std::cout << "     |-longitudinalAcceleration: " << std::endl;
            std::cout << "      |-longitudinalAccelerationValue: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue << std::endl;
            std::cout << "      |-longitudinalAccelerationConfidence: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence << std::endl;
            std::cout << "     |-curvature: " << std::endl;
            std::cout << "      |-curvatureValue: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue << std::endl;
            std::cout << "      |-curvatureConfidence: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureConfidence << std::endl;
            std::cout << "     |-curvatureCalculationMode: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvatureCalculationMode << std::endl;
            std::cout << "     |-yawRate: " << std::endl;
            std::cout << "      |-yawRateValue: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue << std::endl;
            std::cout << "      |-yawRateConfidence: " << cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence << std::endl;

            if(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.accelerationControl)
                std::cout << "     |-accelerationControl(OPTIONAL): present - parsing not yet implemented" << std::endl;
            else
                std::cout << "     |-accelerationControl(OPTIONAL): not present" << std::endl;

            if(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lanePosition)
                std::cout << "     |-lanePosition(OPTIONAL): present - parsing not yet implemented" << std::endl;
            else
                std::cout << "     |-lanePosition(OPTIONAL): not present" << std::endl;

            if(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.steeringWheelAngle)
                std::cout << "     |-steeringWheelAngle(OPTIONAL): present - parsing not yet implemented" << std::endl;
            else
                std::cout << "     |-steeringWheelAngle(OPTIONAL): not present" << std::endl;

            if(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration)
                std::cout << "     |-lateralAcceleration(OPTIONAL): present - parsing not yet implemented" << std::endl;
            else
                std::cout << "     |-lateralAcceleration(OPTIONAL): not present" << std::endl;

            if(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.verticalAcceleration)
                std::cout << "     |-verticalAcceleration(OPTIONAL): present - parsing not yet implemented" << std::endl;
            else
                std::cout << "     |-verticalAcceleration(OPTIONAL): not present" << std::endl;

            if(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.performanceClass)
                std::cout << "     |-performanceClass(OPTIONAL): present - parsing not yet implemented" << std::endl;
            else
                std::cout << "     |-performanceClass(OPTIONAL): not present" << std::endl;

            if(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.cenDsrcTollingZone)
                std::cout << "     |-cenDsrcTollingZone(OPTIONAL): present - parsing not yet implemented" << std::endl;
            else
                std::cout << "     |-cenDsrcTollingZone(OPTIONAL): not present" << std::endl;

            break;
        case(HighFrequencyContainer_PR_rsuContainerHighFrequency):
            std::cout << "   |-choice: " << "HighFrequencyContainer_PR_rsuContainerHighFrequency" << std::endl;
            std::cout << "    |-rsuContainerHighFrequency: " << std::endl;

            if(cam->cam.camParameters.highFrequencyContainer.choice.rsuContainerHighFrequency.protectedCommunicationZonesRSU)
                std::cout << "     |-protectedCommunicationZonesRSU(OPTIONAL): present - parsing not yet implemented" << std::endl;
            else
                std::cout << "     |-protectedCommunicationZonesRSU(OPTIONAL): not present" << std::endl;
            break;
        default:
            std::cout << "Non standard present value" << std::endl;
    }

    if(cam->cam.camParameters.lowFrequencyContainer) {
        std::cout << "  |-lowFrequencyContainer(OPTIONAL): present" << std::endl;
        std::cout << "   |-present: " << ((LowFrequencyContainer*)cam->cam.camParameters.lowFrequencyContainer)->present << std::endl;
        switch (cam->cam.camParameters.lowFrequencyContainer->present) {
            case(LowFrequencyContainer_PR_NOTHING):
                std::cout << "   |-choice: " << "LowFrequencyContainer_PR_NOTHING" << std::endl;
                break;
            case(LowFrequencyContainer_PR_basicVehicleContainerLowFrequency):
                std::cout << "   |-choice: " << "LowFrequencyContainer_PR_basicVehicleContainerLowFrequency" << std::endl;
                std::cout << "    |-basicVehicleContainerLowFrequency: " << std::endl;
                std::cout << "     |-vehicleRole: " << cam->cam.camParameters.lowFrequencyContainer->choice.basicVehicleContainerLowFrequency.vehicleRole << std::endl;
                std::cout << "     |-exteriorLights: BITSTRING - not implemented for displaying in demo" << std::endl;
                std::cout << "     |-pathHistory: A_SEQUENCE_OF - not implemented for displaying in demo" << std::endl;
                break;
            default:
                std::cout << "Non standard present value" << std::endl;
        }
    }
    else
        std::cout << "  |-lowFrequencyContainer(OPTIONAL): not present" << std::endl;

    if(cam->cam.camParameters.specialVehicleContainer) {
        std::cout << "  |-specialVehicleContainer(OPTIONAL): present" << std::endl;
        std::cout << "   |-present: " << cam->cam.camParameters.specialVehicleContainer->present << std::endl;
        switch (cam->cam.camParameters.specialVehicleContainer->present) {
            case(SpecialVehicleContainer_PR_NOTHING):
                std::cout << "   |-choice: " << "SpecialVehicleContainer_PR_NOTHING" << std::endl;
                break;
            case(SpecialVehicleContainer_PR_publicTransportContainer):
                std::cout << "   |-choice: " << "SpecialVehicleContainer_PR_publicTransportContainer" << std::endl;
                std::cout << "    |-publicTransportContainer: content parsing not further implemented in demo" << std::endl;
                break;
            case(SpecialVehicleContainer_PR_specialTransportContainer):
                std::cout << "   |-choice: " << "SpecialVehicleContainer_PR_specialTransportContainer" << std::endl;
                std::cout << "    |-specialTransportContainer: content parsing not further implemented in demo" << std::endl;
                break;
            case(SpecialVehicleContainer_PR_dangerousGoodsContainer):
                std::cout << "   |-choice: " << "SpecialVehicleContainer_PR_dangerousGoodsContainer" << std::endl;
                std::cout << "    |-dangerousGoodsContainer: content parsing not further implemented in demo" << std::endl;
                break;
            case(SpecialVehicleContainer_PR_roadWorksContainerBasic):
                std::cout << "   |-choice: " << "SpecialVehicleContainer_PR_roadWorksContainerBasic" << std::endl;
                std::cout << "    |-roadWorksContainerBasic: content parsing not further implemented in demo" << std::endl;
                break;
            case(SpecialVehicleContainer_PR_rescueContainer):
                std::cout << "   |-choice: " << "SpecialVehicleContainer_PR_rescueContainer" << std::endl;
                std::cout << "    |-rescueContainer: content parsing not further implemented in demo" << std::endl;
                break;
            case(SpecialVehicleContainer_PR_emergencyContainer):
                std::cout << "   |-choice: " << "SpecialVehicleContainer_PR_emergencyContainer" << std::endl;
                std::cout << "    |-emergencyContainer: content parsing not further implemented in demo" << std::endl;
                break;
            case(SpecialVehicleContainer_PR_safetyCarContainer):
                std::cout << "   |-choice: " << "SpecialVehicleContainer_PR_safetyCarContainer" << std::endl;
                std::cout << "    |-safetyCarContainer: content parsing not further implemented in demo" << std::endl;
                break;
            default:
                std::cout << "Non standard present value" << std::endl;
        }
    }
    else
        std::cout << "  |-specialVehicleContainer(OPTIONAL): not present" << std::endl;

    ASN_STRUCT_FREE(asn_DEF_CAM, cam);
    cam = nullptr;
}
void print_poi(void *msg, size_t size){
    std::cout << "--- --- --- POI --- --- ---" << std::endl;
    if( msg && size)
        std::cout << "print_poi(): " << "not yet implemented" << std::endl;
}
void print_spatem(void *msg, size_t size){
    std::cout << "--- --- --- SPATEM --- --- ---" << std::endl;
    if( msg && size)
        std::cout << "print_spatem(): " << "not yet implemented" << std::endl;
}
void print_mapem(void *msg, size_t size){
    std::cout << "--- --- --- MAPEM --- --- ---" << std::endl;
    if( msg && size)
        std::cout << "print_mapem(): " << "not yet implemented" << std::endl;
}
void print_ivim(void *msg, size_t size){
    std::cout << "--- --- --- IVIM --- --- ---" << std::endl;
    if( msg && size)
        std::cout << "print_ivim(): " << "not yet implemented" << std::endl;
}
void print_ev_rsr(void *msg, size_t size){
    std::cout << "--- --- --- EV RSR --- --- ---" << std::endl;
    if( msg && size)
        std::cout << "print_ev_rsr(): " << "not yet implemented" << std::endl;
}
void print_tistpgtransaction(void *msg, size_t size){
    std::cout << "--- --- --- TISTPGTRANSACTION --- --- ---" << std::endl;
    if( msg && size)
        std::cout << "print_tistpgtransaction(): " << "not yet implemented" << std::endl;
}
void print_srem(void *msg, size_t size){
    std::cout << "--- --- --- SREM --- --- ---" << std::endl;
    if( msg && size)
        std::cout << "print_srem(): " << "not yet implemented" << std::endl;
}
void print_ssem(void *msg, size_t size){
    std::cout << "--- --- --- SSEM --- --- ---" << std::endl;
    if( msg && size)
        std::cout << "print_ssem(): " << "not yet implemented" << std::endl;
}
void print_evcsn(void *msg, size_t size){
    std::cout << "--- --- --- EVCSN --- --- ---" << std::endl;
    if( msg && size)
        std::cout << "print_evcsn(): " << "not yet implemented" << std::endl;
}
void print_saem(void *msg, size_t size){
    std::cout << "--- --- --- SAEM --- --- ---" << std::endl;
    if( msg && size)
        std::cout << "print_saem(): " << "not yet implemented" << std::endl;
}
void print_rtcmem(void *msg, size_t size){
    std::cout << "--- --- --- RTCMEM --- --- ---" << std::endl;

    RTCMEM_t *rtcmem = nullptr;

    rtcmem = (RTCMEM_t *) Decoder::decode(&asn_DEF_RTCMEM, msg, size);

    if(rtcmem->header.protocolVersion != 2){
        std::cout << "this sample code only supports version 2 encoding" << std::endl;
        return;
    }

    std::cout << "RTCMEM" << std::endl;
    std::cout << "|-header:" << std::endl;
    std::cout << " |-protocolVersion: " << rtcmem->header.protocolVersion << std::endl;
    std::cout << " |-messageID: " << rtcmem->header.messageID << std::endl;
    std::cout << " |-stationID: " << rtcmem->header.stationID << std::endl;
    std::cout << "|-rtcmc:" << std::endl;
    std::cout << " |-msgCnt: " << rtcmem->rtcmc.msgCnt << std::endl;
    std::cout << " |-rev: " << rtcmem->rtcmc.rev << std::endl;
    if(rtcmem->rtcmc.timeStamp) {
        std::cout << " |-timeStamp: parsing not yet implemented" << std::endl;
        //std::cout << " |-timeStamp: " << rtcmem->rtcmc.timeStamp->? << std::endl;
    }
    if(rtcmem->rtcmc.anchorPoint) {
        std::cout << " |-anchorPoint: parsing not yet implemented" << std::endl;
        //std::cout << " |-anchorPoint: " << rtcmem->rtcmc.anchorPoint->? << std::endl;
    }
    if(rtcmem->rtcmc.rtcmHeader) {
        std::cout << " |-rtcmHeader: parsing not yet implemented" << std::endl;
        //std::cout << " |-rtcmHeader: " << rtcmem->rtcmc.rtcmHeader->? << std::endl;
    }
    std::cout << " |-msgs: has " << rtcmem->rtcmc.msgs.list.size << "elements" << std::endl;
    for (int i = 0 ; i < rtcmem->rtcmc.msgs.list.size; i++) {
        // pointer to OCTET_STRING: rtcmem->rtcmc.msgs.list.array[i]
        // uint8 buffer pointer: rtcmem->rtcmc.msgs.list.array[i]->buf
        // size_t buffer size: rtcmem->rtcmc.msgs.list.array[i]->size

        // output conversion to hex string
        // - performance = "slow"
        // - use hashmap, if needed (https://codereview.stackexchange.com/questions/78535/converting-array-of-bytes-to-the-hex-string-representation)
        std::stringstream ss;
        ss << std::hex;
        for (unsigned int j = 0; j < rtcmem->rtcmc.msgs.list.array[i]->size; ++j)
            ss << std::setw(2) << std::setfill('0') << (int)rtcmem->rtcmc.msgs.list.array[i]->buf[j];
        std::cout << "  |-[" << i << "]: " << ss.str() << std::endl;
    }
    if(rtcmem->rtcmc.regional) {
        std::cout << " |-regional: parsing not yet implemented" << std::endl;
        //std::cout << " |-regional: " << rtcmem->rtcmc.regional->? << std::endl;
    }

    ASN_STRUCT_FREE(asn_DEF_RTCMEM, rtcmem);
    rtcmem = nullptr;
}
