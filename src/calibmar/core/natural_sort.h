
// Adapted Version of https://github.com/scopeInfinity/NaturalSort licensed with
// The MIT License (MIT)
// Copyright (c) 2016 Gagan Kumar(scopeInfinity)

#pragma once

#include <algorithm>
#include <cctype>
#include <vector>

namespace {
  /********** Compare Two Character CaseInsensitive ********/
  template <typename ElementType>
  bool natural_less(const ElementType& lhs, const ElementType& rhs) {
    if (tolower(lhs) < tolower(rhs)) return true;
    return false;
  }

  template <typename ElementType>
  bool is_not_digit(const ElementType& x) {
    return !isdigit(x);
  }

  /********** Compare Two Iterators CaseInsensitive ********/
  template <typename ElementType, typename Iterator>
  struct comp_over_iterator {
    int operator()(const Iterator& lhs, const Iterator& rhs) const {
      if (natural_less<ElementType>(*lhs, *rhs)) return -1;
      if (natural_less<ElementType>(*rhs, *lhs)) return +1;
      return 0;
    }
  };

  /****************************************************
  Comparing two SubString from (Begin,End Iterator each)
  with only digits.
  Usage :
          int compare_number()(\
          FirstNumberBeginIterator,FirstNumberEndIterator,isFirstNumberFractionalPart\
          SecondNumberBeginIterator,SecondNumberEndIterator,isSecondNumberFractionalPart\
          );

  Returns :
          -1  - Number1  <   Number2
           0  - Number1  ==  Number2
           1  - Number1  >   Number2

  ***************************************************/

  template <typename ValueType, typename Iterator>
  struct compare_number {
   private:
    // If Number is Itself fractional Part
    int fractional(Iterator lhsBegin, Iterator lhsEnd, Iterator rhsBegin, Iterator rhsEnd) {
      while (lhsBegin < lhsEnd && rhsBegin < rhsEnd) {
        int local_compare = comp_over_iterator<ValueType, Iterator>()(lhsBegin, rhsBegin);
        if (local_compare != 0) return local_compare;
        lhsBegin++;
        rhsBegin++;
      }
      while (lhsBegin < lhsEnd && *lhsBegin == '0') lhsBegin++;
      while (rhsBegin < rhsEnd && *rhsBegin == '0') rhsBegin++;
      if (lhsBegin == lhsEnd && rhsBegin != rhsEnd)
        return -1;
      else if (lhsBegin != lhsEnd && rhsBegin == rhsEnd)
        return +1;
      else  // lhsBegin==lhsEnd && rhsBegin==rhsEnd
        return 0;
    }
    int non_fractional(Iterator lhsBegin, Iterator lhsEnd, Iterator rhsBegin, Iterator rhsEnd) {
      // Skip Inital Zero's
      while (lhsBegin < lhsEnd && *lhsBegin == '0') lhsBegin++;
      while (rhsBegin < rhsEnd && *rhsBegin == '0') rhsBegin++;

      // Comparing By Length of Both String
      if (lhsEnd - lhsBegin < rhsEnd - rhsBegin) return -1;
      if (lhsEnd - lhsBegin > rhsEnd - rhsBegin) return +1;

      // Equal In length
      while (lhsBegin < lhsEnd) {
        int local_compare = comp_over_iterator<ValueType, Iterator>()(lhsBegin, rhsBegin);
        if (local_compare != 0) return local_compare;
        lhsBegin++;
        rhsBegin++;
      }
      return 0;
    }

   public:
    int operator()(Iterator lhsBegin, Iterator lhsEnd, bool isFractionalPart1, Iterator rhsBegin, Iterator rhsEnd,
                   bool isFractionalPart2) {
      if (isFractionalPart1 && !isFractionalPart2) return true;   // 0<num1<1 && num2>=1
      if (!isFractionalPart1 && isFractionalPart2) return false;  // 0<num2<1 && num1>=1

      // isFractionPart1 == isFactionalPart2
      if (isFractionalPart1)
        return fractional(lhsBegin, lhsEnd, rhsBegin, rhsEnd);
      else
        return non_fractional(lhsBegin, lhsEnd, rhsBegin, rhsEnd);
    }
  };

  /***********************************************************************
  Natural Comparision of Two String using both's (Begin and End Iterator)

  Returns :
          -1  - String1  <   String2
           0  - String1  ==  String2
           1  - String1  >   String2

  Suffix 1 represents for components of 1st String
  Suffix 2 represents for components of 2nd String
  ************************************************************************/

  template <typename ElementType, typename Iterator>
  bool _compare(const Iterator& lhsBegin, const Iterator& lhsEnd, const Iterator& rhsBegin, const Iterator& rhsEnd) {
    Iterator current1 = lhsBegin, current2 = rhsBegin;

    // Flag for Space Found Check
    bool flag_found_space1 = false, flag_found_space2 = false;

    while (current1 != lhsEnd && current2 != rhsEnd) {
      // Ignore More than One Continous Space

      /******************************************
      For HandlingComparision Like
              Hello   9
              Hello  10
              Hello 123
      ******************************************/
      while (flag_found_space1 && current1 != lhsEnd && *current1 == ' ') current1++;
      flag_found_space1 = false;
      if (*current1 == ' ') flag_found_space1 = true;

      while (flag_found_space2 && current2 != rhsEnd && *current2 == ' ') current2++;
      flag_found_space2 = false;
      if (*current2 == ' ') flag_found_space2 = true;

      if (!isdigit(*current1) || !isdigit(*current2)) {
        // Normal comparision if any of character is non digit character
        if (natural_less<ElementType>(*current1, *current2)) return true;
        if (natural_less<ElementType>(*current2, *current1)) return false;
        current1++;
        current2++;
      }
      else {
        /*********************************
        Capture Numeric Part of Both String
        and then using it to compare Both

        ***********************************/
        Iterator last_nondigit1 = std::find_if(current1, lhsEnd, is_not_digit<ElementType>);
        Iterator last_nondigit2 = std::find_if(current2, rhsEnd, is_not_digit<ElementType>);

        int result =
            compare_number<ElementType, Iterator>()(current1, last_nondigit1, (current1 > lhsBegin && *(current1 - 1) == '.'),
                                                    current2, last_nondigit2, (current2 > rhsBegin && *(current2 - 1) == '.'));
        if (result < 0) return true;
        if (result > 0) return false;
        current1 = last_nondigit1;
        current2 = last_nondigit2;
      }
    }

    if (current1 == lhsEnd && current2 == rhsEnd) {
      return false;
    }
    else {
      return current1 == lhsEnd;
    }
  }
}

namespace calibmar {
  namespace natural_sort {
    template <typename String>
    inline bool compare(const String& first, const String& second) {
      return _compare<typename String::value_type, typename String::const_iterator>(first.begin(), first.end(), second.begin(),
                                                                                    second.end());
    }

    template <>
    inline bool compare(char* const& first, char* const& second) {
      char* it1 = first;
      while (*it1 != '\0') it1++;
      char* it2 = second;
      while (*it2 != '\0') it2++;
      return _compare<char, char*>(first, it1, second, it2);
    }
  }
}