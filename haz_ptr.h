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

template<typename T>
struct DefaultDeleter {
    DefaultDeleter() = default;

    void operator()(void *p) {
        T *type_ptr = reinterpret_cast<T *>(p);
        delete type_ptr;
    }
};

class HazPtrSlice {
    static constexpr size_t kMaxSlot = 8;
public:
    HazPtrSlice() = default;

    bool IsInit() const {
        return len_ != 0;
    }

    void Init(std::atomic<uintptr_t> *start, size_t len) {
        protected_ = start;
        len_ = len;
        for (size_t i = len; i < kMaxSlot; i++) {
            map_.set(i, true);
        }
    }

    bool HasFreeSlot() const {
        return !map_.all();
    }

    bool TrySet(uintptr_t ptr, size_t &slot) {
        if (map_.all()) {
            return false;
        }
        for (size_t i = 0; i < len_; i++) {
            if (!map_.test(i)) {
                map_.set(i, true);
                protected_[i].store(std::memory_order_release);
                slot = i;
                return true;
            }
        }
        return false;
    }

    void Unset(size_t slot) {
        map_.set(slot, false);
        protected_[slot].store(std::memory_order_release);
    }

private:
    std::atomic<uintptr_t> *protected_{nullptr};
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
    std::vector<std::atomic<uintptr_t>> protected_;
    std::vector<int> thread_idx_;
    size_t slot_per_thread_;

    static thread_local size_t idx;
    static thread_local HazPtrSlice local_protected_;
    static thread_local std::queue<RetiredBlock> retired_queue_;

public:
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
            RetiredBlock block = retired_queue_.front();
            if (!IsNotProtected((uintptr_t) block.ptr_)) {
                break;
            }
            retired_queue_.pop();
            block.Free();
        }
    }

    bool IsNotProtected(uintptr_t ptr) {
        for (const auto &elem : protected_) {
            if (ptr == elem.load(std::memory_order_acquire)) {
                return false;
            }
        }
        return true;
    }
};

extern HazPtrDomain DEFAULT_HAZPTR_DOMAIN;

class HazPtrHolder {
    constexpr static size_t kImpossibleSlotNum = 1000;
private:
    size_t slot_{kImpossibleSlotNum};
public:
    template<typename T>
    bool Pin(std::atomic<T *> &res) {
        for (;;) {
            T *ptr1 = res.load(std::memory_order_acquire);
            if (!Set(ptr1)) {
                return false;
            }
            T *ptr2 = res.load(std::memory_order_acquire);
            if (ptr1 == ptr2) {
                return true;
            }
        }
    }

    template<typename T>
    bool Set(T *ptr) {
        if (!HazPtrDomain::local_protected_.IsInit()) {
            SetDomainThreadIdx();
            HazPtrDomain::local_protected_.Init(
                    DEFAULT_HAZPTR_DOMAIN.protected_.data() +
                    DEFAULT_HAZPTR_DOMAIN.slot_per_thread_ * HazPtrDomain::idx,
                    DEFAULT_HAZPTR_DOMAIN.slot_per_thread_
            );
        }
        return HazPtrDomain::local_protected_.TrySet((uintptr_t) ptr, slot_);
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
    static void SetDomainThreadIdx() {
        for (size_t i = 0; i < DEFAULT_HAZPTR_DOMAIN.thread_idx_.size(); i++) {
            if (DEFAULT_HAZPTR_DOMAIN.thread_idx_[i] == 0) {
                DEFAULT_HAZPTR_DOMAIN.thread_idx_[i] = 1;
                HazPtrDomain::idx = i;
                return;
            }
        }
        std::cerr << "Thread amount excess limit" << std::endl;
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

thread_local std::queue<RetiredBlock> HazPtrDomain::retired_queue_;

