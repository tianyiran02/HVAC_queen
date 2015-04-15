/*********************************************************************
  This .c file provide Boyer-Moore String Matching Algorithm. Most 
  credit goes to GreeksforGreeks website.

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


/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 max (uint8 a, uint8 b) 
{ return (a > b)? a: b; }

void badCharHeuristic( const char *, int, int badchar[NO_OF_CHARS]);

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
void badCharHeuristic( const char *str, int size, int badchar[NO_OF_CHARS])
{
    int i;
 
    // Initialize all occurrences as -1
    for (i = 0; i < NO_OF_CHARS; i++)
         badchar[i] = -1;
 
    // Fill the actual value of last occurrence of a character
    for (i = 0; i < size; i++)
         badchar[(int) str[i]] = i;
}

short BMsearch(BMStringMatching_t BMMatching)
{
    int m = BMMatching.PatternLength;
    int n = BMMatching.StringLength;
 
    int badchar[NO_OF_CHARS];
 
    /* Fill the bad character array by calling the preprocessing
       function badCharHeuristic() for given pattern */
    badCharHeuristic(BMMatching.PatternAddr, m, badchar);
 
    int s = 0;  // s is shift of the pattern with respect to text
    int k = 0;  // k is the index for the BMatching.Matchingpoint structure
    BMMatching.MatchFlag = 0;
    while(s <= (n - m))
    {
        int j = m-1;
 
        /* Keep reducing index j of pattern while characters of
           pattern and text are matching at this shift s */
        while(j >= 0 && *(BMMatching.PatternAddr + j) == *(BMMatching.StringAddr + (s + j)))
            j--;
 
        /* If the pattern is present at current shift, then index j
           will become -1 after the above loop */
        if (j < 0)
        {
            *(BMMatching.MatchingPoint + (k++)) = s;
            BMMatching.MatchFlag = 1;
 
            /* Shift the pattern so that the next character in text
               aligns with the last occurrence of it in pattern.
               The condition s+m < n is necessary for the case when
               pattern occurs at the end of text */
            s += (s+m < n)? m-badchar[*(BMMatching.StringAddr + (s + m))] : 1;
 
        }
 
        else
            /* Shift the pattern so that the bad character in text
               aligns with the last occurrence of it in pattern. The
               max function is used to make sure that we get a positive
               shift. We may get a negative shift if the last occurrence
               of bad character in pattern is on the right side of the
               current character. */
            s += max(1, j - badchar[*(BMMatching.StringAddr + (s + j))]);
    }
    
    return BMMatching.MatchFlag;
}

