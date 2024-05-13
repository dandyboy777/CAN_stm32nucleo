
#ifndef STACK_PR_H
#define STACK_PR_H

#include "types.h"

//#include "RTL.h"


typedef struct _t_stack
{
volatile    u8* pHead;  //  начало стека
volatile    u8* pTail;  //  конец стека
volatile    u8* pPush;  //  позици€ дл€ записи
volatile    u8* pPop;   //  позици€ дл€ извлечени€
volatile    u32 nVol;   //  объем хранимого
volatile    u32 nSize;  //  размер стека
}t_stack, *h_stack;
//--------------------------------------------------------------------------------

int stk_init    (h_stack hstack, void* pmem, u32 nsize);

//--------------------------------------------------------------------------------

int stk_push_b  (h_stack hstack, u8 byte);
int stk_pop_b   (h_stack hstack, u8 *pbyte);
u16 stk_get_len (h_stack hstack);
int stk_pop_mas(h_stack hstack, u8 *pmas, u16 pmas_len);
int stk_push_mas(h_stack hstack, u8 *pmas, u16 ptr_len);

//--------------------------------------------------------------------------------

#endif
