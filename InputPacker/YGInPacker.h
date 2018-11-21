

#ifndef _YG_IN_PACKER_H_
#define _YG_IN_PACKER_H_

#include "InputPacker.h"

class YGInPacker : public InputPacker
{
  public:
    virtual ~YGInPacker();

    virtual void init(const std::string &caseRootFolder);
};

#endif //_YG_IN_PACKER_H_/