/*******************************************************************************
*
*	TITLE:		teleop.h 
*
*	VERSION:	0.1 (Beta)                           
*
*	DATE:		31-Dec-2007
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This file best viewed with tabs set to four.
*
*				You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or elsewhere
*				without permission. Thanks.
*
*				Copyright �2007-2008 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	Change log:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	31-Dec-2007  0.1  RKW Original
*
*******************************************************************************/
#ifndef _teleop_h
#define _teleop_h

// function prototypes
void Initialization(void);
void Teleop_Init(void);
void Teleop(void);
void Teleop_Spin(void);

#endif