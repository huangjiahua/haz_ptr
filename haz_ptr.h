//
// Created by jiahua on 2019/10/26.
//
// Copyright 2019 Jiahua Huang
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without
// limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#pragma once

#include <utility>
#include <vector>
#include <cstdint>
#include <atomic>
#include <queue>
#include <functional>
#include <bitset>
#include <iostream>
#include <mutex>
#include <cassert>

template<typename T>
struct DefaultDeleter {
    DefaultDeleter() = default;

    void operator()(void *p) {
        T *type_ptr = reinterpret_cast<T *>(p);
        delete type_ptr;
    }
};

struct alignas(128) ProtectedSlot {
    std::atomic<uintptr_t> haz_ptr_{(uintptr_t) nullptr};
};

class HazPtrSlice {
    static constexpr size_t kMaxSlot = 8;
public:
    HazPtrSlice() = default;

    bool IsInit() const {
        return len_ != 0;
    }

    void Init(ProtectedSlot **start, size_t len) {
        protected_ = start;
        len_ = len;
        for (size_t i = len; i < kMaxSlot; i++) {
            map_.set(i);
        }
    }

    bool HasFreeSlot() const {
        return !map_.all();
    }

    bool Empty() const {
        size_t count = map_.count();
        return (count == kMaxSlot - len_);
    }

    bool TrySet(uintptr_t ptr, size_t &slot) {
        if (map_.all()) {
            return false;
        }
        for (size_t i = 0; i < len_; i++) {
            if (!map_.test(i)) {
                map_.set(i);
                protected_[i]->haz_ptr_.store(ptr, std::memory_order_release);
                slot = i;
                return true;
            }
        }
        return false;
    }

    void Replace(uintptr_t ptr, size_t slot) {
        protected_[slot]->haz_ptr_.store(ptr, std::memory_order_release);
    }

    void Unset(size_t slot, bool no_need_to_publish = false) {
        if (slot < kMaxSlot) {
            map_.reset(slot);
            if (!no_need_to_publish) {
                protected_[slot]->haz_ptr_.store((uintptr_t) nullptr, std::memory_order_release);
            }
        }
    }

private:
    ProtectedSlot **protected_{nullptr};
    std::bitset<kMaxSlot> map_;
    size_t len_{0};
};

struct RetiredBlock {
    void *ptr_;
    std::function<void(void *)> deleter_;

    RetiredBlock() : ptr_(nullptr), deleter_() {}

    RetiredBlock(void *p, std::function<void(void *)> d) : ptr_(p), deleter_(std::move(d)) {}

    void Free() {
        deleter_(ptr_);
    }
};


class HazPtrHolder;

class HazPtrDomain {
    constexpr static size_t kMaxRetiredLen = 68;
    constexpr static size_t kMustTryFree = 64;

    friend class HazPtrHolder;

private:
    std::mutex mut_;
    std::vector<ProtectedSlot *> protected_;
    std::vector<int> thread_idx_;
    size_t slot_per_thread_;
    size_t thread_cnt_;

    static thread_local size_t idx_;
    static thread_local HazPtrSlice local_protected_;
    static thread_local std::queue<RetiredBlock> retired_queue_;

public:
    void Init(size_t thread_cnt = 16, size_t quota = 2) {
        thread_cnt_ = thread_cnt + 2;
        protected_.resize(thread_cnt_ * quota, nullptr);
        for (size_t i = 0; i < thread_cnt_ * quota; i++) {
            protected_[i] = new ProtectedSlot;
        }
        thread_idx_.resize(thread_cnt_, 0);
        slot_per_thread_ = quota;
    }

    template<typename T>
    void PushRetired(T *ptr) {
        PushRetired(ptr, DefaultDeleter<T>());
    }

    template<typename T>
    void PushRetired(T *ptr, const std::function<void(void *)> &deleter) {
        if (!ptr) {
            return;
        }
        if (retired_queue_.size() >= kMaxRetiredLen) {
            TryFreeSomeBlock();
        }
        retired_queue_.push(RetiredBlock((void *) ptr, deleter));
    }

private:
    void TryFreeSomeBlock() {
        if (retired_queue_.size() < kMaxRetiredLen) {
            return;
        }

        std::array<uintptr_t, kMustTryFree> ptrs{};
        for (auto &ptr : ptrs) ptr = (uintptr_t) nullptr;
        std::array<RetiredBlock, kMustTryFree> blocks{};

        for (size_t i = 0; i < kMustTryFree && !retired_queue_.empty(); i++) {
            RetiredBlock block = retired_queue_.front();
            retired_queue_.pop();
            ptrs[i] = (uintptr_t) block.ptr_;
            blocks[i] = block;
        }

        std::bitset<kMustTryFree> res = IsNotProtected(ptrs);

        if (res.none()) {
            for (RetiredBlock &block: blocks) {
                block.Free();
            }
        } else {
            for (size_t i = 0; i < kMustTryFree; i++) {
                if (!res.test(i)) {
                    blocks[i].Free();
                } else {
                    retired_queue_.push(blocks[i]);
                }
            }
        }
    }

    bool IsNotProtected(uintptr_t ptr) {
        for (size_t i = 0; i < ProtectedSize(); i++) {
            if (ptr == protected_[i]->haz_ptr_.load(std::memory_order_acquire)) {
                return false;
            }
        }
        return true;
    }

    std::bitset<kMustTryFree> IsNotProtected(std::array<uintptr_t, kMustTryFree> &ptrs) {
        std::bitset<kMustTryFree> ret;
        for (size_t i = 0; i < ProtectedSize(); i++) {
            uint64_t target = protected_[i]->haz_ptr_.load(std::memory_order_acquire);
            for (size_t j = 0; j < kMustTryFree; j++) {
                assert(ptrs[j]);
                if (ptrs[j] == target) {
                    ret.set(j);
                }
            }
        }
        return ret;
    }

    size_t ProtectedSize() const {
        return thread_cnt_ * slot_per_thread_;
    }
};

extern HazPtrDomain DEFAULT_HAZPTR_DOMAIN;


class HazPtrHolder {
    constexpr static size_t kImpossibleSlotNum = 1000;
private:
    size_t slot_;
    void *pinned_;
public:
    HazPtrHolder() : slot_(kImpossibleSlotNum) {
    }

    template<typename T>
    T *Pin(std::atomic<T *> &res) {
        for (;;) {
            T *ptr1 = res.load(std::memory_order_acquire);

            if (!ptr1) {
                return nullptr;
            }

            if (!Set(ptr1)) {
                std::cerr << "This thread can only protect " << DEFAULT_HAZPTR_DOMAIN.slot_per_thread_ << " pointers"
                          << std::endl;
                exit(1);
            }
            T *ptr2 = res.load(std::memory_order_acquire);
            if (ptr1 == ptr2) {
                pinned_ = (void *) ptr1;
                return ptr1;
            }
            Reset();
        }
    }

    template <typename T>
    T *Repin(std::atomic<T *> &res) {
        if (slot_ == kImpossibleSlotNum) {
            return Pin(res);
        }

        for (;;) {
            T *ptr1 = res.load(std::memory_order_acquire);

            if (!ptr1) {
                Reset();
                return nullptr;
            }

            Replace(ptr1);

            T *ptr2 = res.load(std::memory_order_acquire);
            if (ptr1 == ptr2) {
                pinned_ = (void*) ptr1;
                return ptr1;
            }
        }
    }

    template<typename T>
    T *Get() const {
        return (T *) pinned_;
    }

    inline void Reset() {
        bool no_need_to_publish = (pinned_ == nullptr);
        HazPtrDomain::local_protected_.Unset(slot_, no_need_to_publish);
        pinned_ = nullptr;
        slot_ = kImpossibleSlotNum;
    }

    ~HazPtrHolder() {
        if (slot_ != kImpossibleSlotNum) {
            Reset();
        }
    }

private:
    template<typename T>
    bool Set(T *ptr) {
        if (!HazPtrDomain::local_protected_.IsInit()) {
            SetDomainThreadIdx();
            HazPtrDomain::local_protected_.Init(
                    DEFAULT_HAZPTR_DOMAIN.protected_.data() +
                    DEFAULT_HAZPTR_DOMAIN.slot_per_thread_ * HazPtrDomain::idx_,
                    DEFAULT_HAZPTR_DOMAIN.slot_per_thread_
            );
        }
        return HazPtrDomain::local_protected_.TrySet((uintptr_t) ptr, slot_);
    }

    template <typename T>
    void Replace(T *ptr) {
        return HazPtrDomain::local_protected_.Replace((uintptr_t) ptr, slot_);
    }

    static void SetDomainThreadIdx() {
        std::lock_guard<std::mutex> lk(DEFAULT_HAZPTR_DOMAIN.mut_);
        for (size_t i = 0; i < DEFAULT_HAZPTR_DOMAIN.thread_idx_.size(); i++) {
            if (DEFAULT_HAZPTR_DOMAIN.thread_idx_[i] == 0) {
                DEFAULT_HAZPTR_DOMAIN.thread_idx_[i] = 1;
                HazPtrDomain::idx_ = i;
                return;
            }
        }
        std::cerr << "Thread amount excess limit" << std::endl;
        exit(1);
    }
};

template<typename T>
inline void HazPtrRetire(T *ptr, const std::function<void(void *)> &deleter) {
    DEFAULT_HAZPTR_DOMAIN.PushRetired(ptr, deleter);
}

template<typename T>
inline void HazPtrRetire(T *ptr) {
    DEFAULT_HAZPTR_DOMAIN.PushRetired(ptr);
}

inline void HazPtrInit() {
    DEFAULT_HAZPTR_DOMAIN.Init();
}

inline void HazPtrInit(size_t thread_cnt) {
    DEFAULT_HAZPTR_DOMAIN.Init(thread_cnt);
}

inline void HazPtrInit(size_t thread_cnt, size_t quota_per_thread) {
    DEFAULT_HAZPTR_DOMAIN.Init(thread_cnt, quota_per_thread);
}

#define ENABLE_LOCAL_DOMAIN HazPtrDomain DEFAULT_HAZPTR_DOMAIN;\
                            thread_local size_t HazPtrDomain::idx_;\
                            thread_local HazPtrSlice HazPtrDomain::local_protected_;\
                            thread_local std::queue<RetiredBlock> HazPtrDomain::retired_queue_;

