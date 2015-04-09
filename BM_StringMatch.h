/*********************************************************************
  Filename:       BM_StringMatch.h

  @Descrip work with BM_StringMatch.c file. provide Boyer-Moore String
  Matching Algoritm

  @Author Yiran Tian 2015/4/9
*********************************************************************/

#ifndef BM_STRINGMATCH_H
#define BM_STRINGMATCH_H


typedef struct
{
  int            PatternLength;
  int            StringLength;
  uint8          *StringAddr;
  uint8          *PatternAddr;
  uint8          *MatchingPoint;
} BMStringMatching_t;

void preBmBc(uint8 *, int , uint8 *);
void suffixes(uint8 *, int , uint8 *);
void preBmGs(uint8 *, int , uint8 *, int, uint8 *); 
BMStringMatching_t BM(BMStringMatching_t );
#endif