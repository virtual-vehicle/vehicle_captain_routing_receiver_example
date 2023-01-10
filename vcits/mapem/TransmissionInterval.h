/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/cdd_ts102894_2/ITS-Container.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/mapem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=MAPEM`
 */

#ifndef	_TransmissionInterval_H_
#define	_TransmissionInterval_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum TransmissionInterval {
	TransmissionInterval_oneMilliSecond	= 1,
	TransmissionInterval_tenSeconds	= 10000
} e_TransmissionInterval;

/* TransmissionInterval */
typedef long	 TransmissionInterval_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TransmissionInterval;
asn_struct_free_f TransmissionInterval_free;
asn_struct_print_f TransmissionInterval_print;
asn_constr_check_f TransmissionInterval_constraint;
ber_type_decoder_f TransmissionInterval_decode_ber;
der_type_encoder_f TransmissionInterval_encode_der;
xer_type_decoder_f TransmissionInterval_decode_xer;
xer_type_encoder_f TransmissionInterval_encode_xer;
oer_type_decoder_f TransmissionInterval_decode_oer;
oer_type_encoder_f TransmissionInterval_encode_oer;
per_type_decoder_f TransmissionInterval_decode_uper;
per_type_encoder_f TransmissionInterval_encode_uper;
per_type_decoder_f TransmissionInterval_decode_aper;
per_type_encoder_f TransmissionInterval_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _TransmissionInterval_H_ */
#include <asn_internal.h>
