/*********************************************************************
  This .c file provide Boyer-Moore String Matching Algorithm. Most 
  credit goes to ESMAJ website.

  Coupling with other module description:

  @Author Yiran Tian 2015/4/9
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <math.h>
#include <hal_types.h>
#include <OSAL.h>


#include "BM_StringMatch.h"

/*********************************************************************
 * MACROS
 */

#ifndef BMmax
#define BMmax(a,b) ((a) > (b) ? (a) : (b)) 
#endif

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

void preBmBc(uint8 *x, int m, uint8 *bmBc) 
{
   int i;
 
   for (i = 0; i < m; ++i)
      bmBc[i] = m;
   for (i = 0; i < m - 1; ++i)
      bmBc[x[i]] = m - i - 1;
}
 
 
void suffixes(uint8 *x, int m, uint8 *suff) 
{
   int f, g, i;
 
   suff[m - 1] = m;
   g = m - 1;
   for (i = m - 2; i >= 0; --i) {
      if (i > g && suff[i + m - 1 - f] < i - g)
         suff[i] = suff[i + m - 1 - f];
      else {
         if (i < g)
            g = i;
         f = i;
         while (g >= 0 && x[g] == x[g + m - 1 - f])
            --g;
         suff[i] = f - g;
      }
   }
}
 
void preBmGs(uint8 *x, int m, uint8 *bmGs, int n, uint8 *suff) 
{
   int i, j;
   //int suff[n] = {0};
   
 
   suffixes(x, m, suff);
 
   for (i = 0; i < m; ++i)
      bmGs[i] = m;
   j = 0;
   for (i = m - 1; i >= 0; --i)
      if (suff[i] == i + 1)
         for (; j < m - 1 - i; ++j)
            if (bmGs[j] == m)
               bmGs[j] = m - 1 - i;
   for (i = 0; i <= m - 2; ++i)
      bmGs[m - 1 - suff[i]] = m - 1 - i;
}
 
 
BMStringMatching_t BM(BMStringMatching_t BMmatching) 
{
   int i, j;
   uint8 *x = BMmatching.StringAddr;
   int m = BMmatching.StringLength;
   uint8 *y =BMmatching.PatternAddr;
   int n = BMmatching.PatternLength;
   
   uint8 *bmGs = osal_mem_alloc(n);
   uint8 *bmBc = osal_mem_alloc(m);
   uint8 *suff = osal_mem_alloc(n);
   
   osal_memset(suff,'0',sizeof(suff));
   osal_memset(bmGs,'0',sizeof(bmGs));
   osal_memset(bmBc,'0',sizeof(bmBc));
    
   /* Preprocessing */
   preBmGs(x, m, bmGs, n, suff);
   preBmBc(x, m, bmBc);
 
   /* Searching */
   j = 0;
   while (j <= n - m) {
      for (i = m - 1; i >= 0 && x[i] == y[i + j]; --i);
      if (i < 0) {
         *(BMmatching.MatchingPoint++) = j;
         j += bmGs[0];
      }
      else
         j +=  BMmax(bmGs[i],(bmBc[y[i + j]] - m + 1 + i));
   }
   
   // free memery
   osal_memset(suff,'0',sizeof(suff));
   osal_memset(bmGs,'0',sizeof(bmGs));
   osal_memset(bmBc,'0',sizeof(bmBc));
   osal_mem_free(bmBc);
   osal_mem_free(bmGs);
   osal_mem_free(suff);
   
   return BMmatching;
}