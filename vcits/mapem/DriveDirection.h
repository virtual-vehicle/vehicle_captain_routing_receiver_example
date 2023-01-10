/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/cdd_ts102894_2/ITS-Container.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/mapem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=MAPEM`
 */

#ifndef	_DriveDirection_H_
#define	_DriveDirection_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DriveDirection {
	DriveDirection_forward	= 0,
	DriveDirection_backward	= 1,
	DriveDirection_unavailable	= 2
} e_DriveDirection;

/* DriveDirection */
typedef long	 DriveDirection_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DriveDirection;
asn_struct_free_f DriveDirection_free;
asn_struct_print_f DriveDirection_print;
asn_constr_check_f DriveDirection_constraint;
ber_type_decoder_f DriveDirection_decode_ber;
der_type_encoder_f DriveDirection_encode_der;
xer_type_decoder_f DriveDirection_decode_xer;
xer_type_encoder_f DriveDirection_encode_xer;
oer_type_decoder_f DriveDirection_decode_oer;
oer_type_encoder_f DriveDirection_encode_oer;
per_type_decoder_f DriveDirection_decode_uper;
per_type_encoder_f DriveDirection_encode_uper;
per_type_decoder_f DriveDirection_decode_aper;
per_type_encoder_f DriveDirection_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _DriveDirection_H_ */
#include <asn_internal.h>