/*
 * Utility routines.
 *
 * Copyright (C) 2006 Denys Vlasenko
 *
 * Licensed under GPLv2, see file LICENSE in this source tree.
 */

/* We do not include libbb.h - #define makedev() is there! */

#ifdef __GLIBC__
/* At least glibc has horrendously large inline for this, so wrap it */
/* uclibc people please check - do we need "&& !__UCLIBC__" above? */

/* suppress gcc "no previous prototype" warning */
unsigned long long FAST_FUNC bb_makedev(unsigned int major, unsigned int minor);
unsigned long long FAST_FUNC bb_makedev(unsigned int major, unsigned int minor)
{
	return makedev(major, minor);
}
#endif
