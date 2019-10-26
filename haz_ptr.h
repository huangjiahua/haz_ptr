//
// Created by jiahua on 2019/10/26.
//

#pragma once

#include <vector>
#include <cstdint>
#include <atomic>

template<typename T>
struct DefaultDeleter {
    void operator()(void *p) {
        T *type_ptr = reinterpret_cast<T *>(p);
        delete type_ptr;
    }
};

class HazPtrSlice {
private:
    std::atomic<uintptr_t> *protected_;
    size_t len_;
};


class HazPtrDomain {
private:
    std::vector<std::atomic<uintptr_t>> protected_;

    static thread_local size_t idx;
    static thread_local HazPtrSlice local_protected_;
};
