/*********************************************************************
  Filename:       BM_StringMatch.h

  @Descrip work with BM_StringMatch.c file. provide Boyer-Moore String
  Matching Algoritm

  @Author Yiran Tian 2015/4/9
*********************************************************************/

#ifndef BM_STRINGMATCH_H
#define BM_STRINGMATCH_H

#define NO_OF_CHARS 151

typedef struct
{
  int            PatternLength;
  int            StringLength;
  char           *StringAddr;
  char           *PatternAddr;
  uint8          *MatchingPoint;
  short          MatchFlag;
} BMStringMatching_t;

short BMsearch(BMStringMatching_t);

#endif