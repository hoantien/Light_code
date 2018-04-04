/*-----------------------------------------------------------------------
//
// Proprietary Information of Elliptic Technologies
// Copyright (C) 2002-2012, all rights reserved
// Elliptic Technologies Inc.
//
// As part of our confidentiality  agreement, Elliptic  Technologies and
// the Company, as  a  Receiving Party, of  this  information  agrees to
// keep strictly  confidential  all Proprietary Information  so received
// from Elliptic  Technologies. Such Proprietary Information can be used
// solely for  the  purpose  of evaluating  and/or conducting a proposed
// business  relationship  or  transaction  between  the  parties.  Each
// Party  agrees  that  any  and  all  Proprietary  Information  is  and
// shall remain confidential and the property of Elliptic  Technologies.
// The  Company  may  not  use  any of  the  Proprietary  Information of
// Elliptic  Technologies for any purpose other  than  the  above-stated
// purpose  without the prior written consent of Elliptic  Technologies.
//
*/

#ifndef ELPSOFT_ERROR_H_
#define ELPSOFT_ERROR_H_

enum {
   ELPEOK=0,
   ELPEINPROGRESS, /* non-fatal */
   ELPEERR,
   ELPEMEM,
   ELPEOVF,
   ELPEOVR,
   ELPEINV,
   ELPEALGMISSING,
   ELPEALGTEST,
   ELPEPRIVATEKEY,
   ELPEAUTHFAILED,
};

#endif /* ELPSOFT_ERROR_H_ */
