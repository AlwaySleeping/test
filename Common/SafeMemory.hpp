
//TODO
#ifndef __SAFE_MEMORY_H__
#define __SAFE_MEMORY_H__

#include <new>

// simple new, return nullptr if failed
#ifndef mnew
#define mnew                new(std::nothrow)
#endif

// safe delete
#ifndef mdelete
#define mdelete(p)          { delete (p); (p) = nullptr; }
#endif

// safe delete array
#ifndef madelete
#define madelete(p)          { delete [] (p); (p) = nullptr; }
#endif

// safe close
#ifndef mclose
#define mclose(p)           { if (nullptr != (p)) {fclose(p); (p) = nullptr;} }
#endif

// class for managing memory (delete memory automatically)
template<class T>
class Mem_
{
 public:
    Mem_()
    {
        pData = nullptr;
        nSize = 0;
    }
    ~Mem_()
    {
        release();
    }
    T *operator [](const int n)
    {
        mdelete(pData);
        if (0 < n)
        {
            pData = mnew T[n];
        }
        nSize = (nullptr == pData) ? 0 : n;
        return pData;
    }
    T *data(void)
    {
        return pData;
    }
    const T *data(void) const
    {
        return pData;
    }
    int size(void) const
    {
        return nSize;
    }
    void release(T* &p)
    {
        nSize = 0;
        mdelete(pData);
        if (nullptr != p)
        {
            p = nullptr;
        }
    }
    void release(void)
    {
        nSize = 0;
        mdelete(pData);
    }
 private:
    T *pData;
    int nSize;
};

typedef Mem_<unsigned char> Mem;
typedef Mem_<int> Memn;
typedef Mem_<bool> Memb;
typedef Mem_<char> Memc;
typedef Mem_<float> Memf;

#endif // __SAFE_MEMORY_H__
