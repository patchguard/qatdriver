/*
 * Automatically generated by asn1_compiler.  Do not edit
 *
 * ASN.1 parser for qat_rsaprivkey
 */
#ifdef QAT_PKE_OLD_SUPPORTED
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,0)
#include <linux/asn1_decoder.h>

extern const struct asn1_decoder qat_rsaprivkey_decoder;

extern int qat_rsa_get_d(void *, size_t, unsigned char, const void *, size_t);
extern int qat_rsa_get_e(void *, size_t, unsigned char, const void *, size_t);
extern int qat_rsa_get_n(void *, size_t, unsigned char, const void *, size_t);
#endif
#endif