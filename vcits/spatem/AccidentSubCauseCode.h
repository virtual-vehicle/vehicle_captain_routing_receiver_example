/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/cdd_ts102894_2/ITS-Container.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/spatem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=SPATEM`
 */

#ifndef	_AccidentSubCauseCode_H_
#define	_AccidentSubCauseCode_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum AccidentSubCauseCode {
	AccidentSubCauseCode_unavailable	= 0,
	AccidentSubCauseCode_multiVehicleAccident	= 1,
	AccidentSubCauseCode_heavyAccident	= 2,
	AccidentSubCauseCode_accidentInvolvingLorry	= 3,
	AccidentSubCauseCode_accidentInvolvingBus	= 4,
	AccidentSubCauseCode_accidentInvolvingHazardousMaterials	= 5,
	AccidentSubCauseCode_accidentOnOppositeLane	= 6,
	AccidentSubCauseCode_unsecuredAccident	= 7,
	AccidentSubCauseCode_assistanceRequested	= 8
} e_AccidentSubCauseCode;

/* AccidentSubCauseCode */
typedef long	 AccidentSubCauseCode_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_AccidentSubCauseCode;
asn_struct_free_f AccidentSubCauseCode_free;
asn_struct_print_f AccidentSubCauseCode_print;
asn_constr_check_f AccidentSubCauseCode_constraint;
ber_type_decoder_f AccidentSubCauseCode_decode_ber;
der_type_encoder_f AccidentSubCauseCode_encode_der;
xer_type_decoder_f AccidentSubCauseCode_decode_xer;
xer_type_encoder_f AccidentSubCauseCode_encode_xer;
oer_type_decoder_f AccidentSubCauseCode_decode_oer;
oer_type_encoder_f AccidentSubCauseCode_encode_oer;
per_type_decoder_f AccidentSubCauseCode_decode_uper;
per_type_encoder_f AccidentSubCauseCode_encode_uper;
per_type_decoder_f AccidentSubCauseCode_decode_aper;
per_type_encoder_f AccidentSubCauseCode_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _AccidentSubCauseCode_H_ */
#include <asn_internal.h>