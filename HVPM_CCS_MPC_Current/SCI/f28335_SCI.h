#ifndef __F28335_SCI_H
#define __F28335_SCI_H

void scib_fifo_init(void);
void scib_xmit(int a);
void scib_xmit_float(float *pdata,int num);

#endif
