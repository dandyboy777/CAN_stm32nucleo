#include "stack_pr.h"


#define EINVAL 1
#define EOVERFLOW 2
#define ENOSPC 3

int stk_init(h_stack hstack, void* pmem, u32 nsize)
{
    if (hstack == NULL)
        return -EINVAL;
    if ((pmem == NULL) || (nsize == 0))
        return -EINVAL;

    hstack->nSize = nsize;
    hstack->nVol = 0;
    hstack->pHead = pmem;
    hstack->pPush = pmem;
    hstack->pPop = pmem;
    hstack->pTail = (u8*) pmem + nsize;

    return 0;
}


//--------------------------------------------------------------------------------

//void __swi(14) _push(h_stack hstack, u8 byte);
//void __swi(15) _pop (h_stack hstack, u8* pbyte);
//void __swi(17) _len (h_stack hstack, u16* len);
void  _push(h_stack hstack, u8 byte);
void  _pop (h_stack hstack, u8* pbyte);
void  _len (h_stack hstack, u16* len);
//--------------------------------------------------------------------------------

void /*__SWI_14*/_push(h_stack hstack, u8 byte)
{
    *(hstack->pPush) = byte;
    hstack->pPush++;

    hstack->nVol++;

    if (hstack->pPush >= hstack->pTail)
        hstack->pPush = hstack->pHead;
}
//--------------------------------------------------------------------------------

void /*__SWI_15*/_pop(h_stack hstack, u8* pbyte)
{
    *pbyte = *(hstack->pPop);
    hstack->pPop++;

    hstack->nVol--;

    if (hstack->pPop >= hstack->pTail)
        hstack->pPop = hstack->pHead;
}

void /*__SWI_17*/ _len(h_stack hstack, u16 *len)
{
    (*len) = hstack->nVol;
}

//--------------------------------------------------------------------------------

u16 stk_get_len(h_stack hstack)
{
    u16 len;
    _len(hstack, &len);
    return len;
}

int stk_push_b(h_stack hstack, u8 byte)
{
    if (hstack == NULL)
        return -EINVAL;
    if (hstack->nVol >= hstack->nSize)
        return -EOVERFLOW;

    _push(hstack, byte);

    return 0;
}

//--------------------------------------------------------------------------------

int stk_push_mas(h_stack hstack, u8 *pmas, u16 ptr_len)
{
    if (hstack == NULL)
        return -EINVAL;
    if (hstack->nVol >= hstack->nSize)
        return -EOVERFLOW;
    while (ptr_len--)
    {
        _push(hstack, *pmas);
        pmas++;
    };
    return 0;
}

//--------------------------------------------------------------------------------

int stk_pop_b(h_stack hstack, u8 *pbyte)
{
    if (hstack == NULL)
        return -EINVAL;
    if (hstack->nVol == 0)
        return -ENOSPC;

    _pop(hstack, pbyte);

    return 0;
}

int stk_pop_mas(h_stack hstack, u8 *pmas, u16 pmas_len)
{
    u16 k = 0;
    if (hstack == NULL)
        return -EINVAL;
    if (hstack->nVol == 0)
        return 0;
    if (pmas_len == 0)
        return 0;
    do
    {
        _pop(hstack, pmas);
        pmas++;
        k++;
        if (hstack->nVol == 0)
            break;
    } while (--pmas_len);
    return k;
}
//--------------------------------------------------------------------------------
