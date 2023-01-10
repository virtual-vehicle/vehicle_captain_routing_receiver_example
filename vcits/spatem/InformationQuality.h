/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/cdd_ts102894_2/ITS-Container.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/spatem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=SPATEM`
 */

#ifndef	_InformationQuality_H_
#define	_InformationQuality_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum InformationQuality {
	InformationQuality_unavailable	= 0,
	InformationQuality_lowest	= 1,
	InformationQuality_highest	= 7
} e_InformationQuality;

/* InformationQuality */
typedef long	 InformationQuality_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_InformationQuality_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_InformationQuality;
asn_struct_free_f InformationQuality_free;
asn_struct_print_f InformationQuality_print;
asn_constr_check_f InformationQuality_constraint;
ber_type_decoder_f InformationQuality_decode_ber;
der_type_encoder_f InformationQuality_encode_der;
xer_type_decoder_f InformationQuality_decode_xer;
xer_type_encoder_f InformationQuality_encode_xer;
oer_type_decoder_f InformationQuality_decode_oer;
oer_type_encoder_f InformationQuality_encode_oer;
per_type_decoder_f InformationQuality_decode_uper;
per_type_encoder_f InformationQuality_encode_uper;
per_type_decoder_f InformationQuality_decode_aper;
per_type_encoder_f InformationQuality_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _InformationQuality_H_ */
#include <asn_internal.h>
