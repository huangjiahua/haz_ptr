//
// Created by jiahua on 2019/10/26.
//

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
                protected_[i]->haz_ptr_.store(std::memory_order_release);
                slot = i;
                return true;
            }
        }
        return false;
    }

    void Unset(size_t slot) {
        if (slot < kMaxSlot) {
            map_.reset(slot);
            protected_[slot]->haz_ptr_.store(std::memory_order_release);
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

    RetiredBlock(void *p, std::function<void(void *)> d) : ptr_(p), deleter_(std::move(d)) {}

    void Free() {
        deleter_(ptr_);
    }
};


class HazPtrHolder;

class HazPtrDomain {
    constexpr static size_t kMaxRetiredLen = 20;
    constexpr static size_t kMustTryFree = 5;

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
        for (size_t i = 0; i < kMustTryFree; i++) {
            RetiredBlock block = retired_queue_.front();
            retired_queue_.pop();
            if (IsNotProtected((uintptr_t) block.ptr_)) {
                block.Free();
            } else {
                retired_queue_.push(block);
            }
        }


        for (;;) {
            if (retired_queue_.empty()) {
                return;
            }
            RetiredBlock block = retired_queue_.front();
            if (!IsNotProtected((uintptr_t) block.ptr_)) {
                break;
            }
            retired_queue_.pop();
            block.Free();
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

    size_t ProtectedSize() const {
        return thread_cnt_ * slot_per_thread_;
    }
};

extern HazPtrDomain DEFAULT_HAZPTR_DOMAIN;


class HazPtrHolder {
    constexpr static size_t kImpossibleSlotNum = 1000;
private:
    size_t slot_;
public:
    HazPtrHolder() : slot_(kImpossibleSlotNum) {
    }

    template<typename T>
    T *Pin(std::atomic<T *> &res) {
        for (;;) {
            T *ptr1 = res.load(std::memory_order_acquire);
            if (!Set(ptr1)) {
                std::cerr << "This thread can only protect " << DEFAULT_HAZPTR_DOMAIN.slot_per_thread_ << " pointers"
                          << std::endl;
                exit(1);
            }
            T *ptr2 = res.load(std::memory_order_acquire);
            if (ptr1 == ptr2) {
                return ptr1;
            }
            Reset();
        }
    }

    void Reset() {
        HazPtrDomain::local_protected_.Unset(slot_);
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
void HazPtrRetire(T *ptr, const std::function<void(void *)> &deleter) {
    DEFAULT_HAZPTR_DOMAIN.PushRetired(ptr, deleter);
}

template<typename T>
void HazPtrRetire(T *ptr) {
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

