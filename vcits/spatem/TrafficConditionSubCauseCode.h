/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/cdd_ts102894_2/ITS-Container.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/spatem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=SPATEM`
 */

#ifndef	_TrafficConditionSubCauseCode_H_
#define	_TrafficConditionSubCauseCode_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum TrafficConditionSubCauseCode {
	TrafficConditionSubCauseCode_unavailable	= 0,
	TrafficConditionSubCauseCode_increasedVolumeOfTraffic	= 1,
	TrafficConditionSubCauseCode_trafficJamSlowlyIncreasing	= 2,
	TrafficConditionSubCauseCode_trafficJamIncreasing	= 3,
	TrafficConditionSubCauseCode_trafficJamStronglyIncreasing	= 4,
	TrafficConditionSubCauseCode_trafficStationary	= 5,
	TrafficConditionSubCauseCode_trafficJamSlightlyDecreasing	= 6,
	TrafficConditionSubCauseCode_trafficJamDecreasing	= 7,
	TrafficConditionSubCauseCode_trafficJamStronglyDecreasing	= 8
} e_TrafficConditionSubCauseCode;

/* TrafficConditionSubCauseCode */
typedef long	 TrafficConditionSubCauseCode_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TrafficConditionSubCauseCode;
asn_struct_free_f TrafficConditionSubCauseCode_free;
asn_struct_print_f TrafficConditionSubCauseCode_print;
asn_constr_check_f TrafficConditionSubCauseCode_constraint;
ber_type_decoder_f TrafficConditionSubCauseCode_decode_ber;
der_type_encoder_f TrafficConditionSubCauseCode_encode_der;
xer_type_decoder_f TrafficConditionSubCauseCode_decode_xer;
xer_type_encoder_f TrafficConditionSubCauseCode_encode_xer;
oer_type_decoder_f TrafficConditionSubCauseCode_decode_oer;
oer_type_encoder_f TrafficConditionSubCauseCode_encode_oer;
per_type_decoder_f TrafficConditionSubCauseCode_decode_uper;
per_type_encoder_f TrafficConditionSubCauseCode_encode_uper;
per_type_decoder_f TrafficConditionSubCauseCode_decode_aper;
per_type_encoder_f TrafficConditionSubCauseCode_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _TrafficConditionSubCauseCode_H_ */
#include <asn_internal.h>
