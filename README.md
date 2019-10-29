# haz_ptr

A simple hazard pointer implementation.

*Huang Jiahua*

## What is "hazard pointers"?

> In a [multithreaded](https://en.wikipedia.org/wiki/Thread_(computer_science)) [computing](https://en.wikipedia.org/wiki/Computer_science) environment, **hazard pointers** are one approach to solving the problems posed by [dynamic memory management](https://en.wikipedia.org/wiki/Dynamic_memory_management) of the nodes in a [lock-free](https://en.wikipedia.org/wiki/Non-blocking_algorithm) [data structure](https://en.wikipedia.org/wiki/Data_structure). These problems generally arise only in environments that don't have [automatic garbage collection](https://en.wikipedia.org/wiki/Automatic_garbage_collection). 
>
> By [WikiPedia](https://en.wikipedia.org/wiki/Hazard_pointer)

## How to Use?

- For now, some global variables should be defined to use because this implementation needs a global state. You can use the macro in the header file or you can build the `haz_ptr.cpp` to your binary.

```c++
ENABLE_LOCAL_DOMAIN // defines all needed variables
```

- You should initialize the global state.

```c++
HazPtrInit(x, y); // Support up to x threads with y pinned slot per thread
// By default, x = 16 and y = 2
```

- The hazard pointer is used to protect some atomic pointers being freed by other threads.

```c++
std::atomic<MyType *> atomic_ptr;
atomic_ptr.store(new MyType);
//...
HazPointerHolder h;
MyType *pinned = h.Pin(atomic_ptr); // passed by ref
DoWhatEverYouWant(pinned);
```

- If you want to free some dynamic pointers, hazard pointer defer the destruction until no thread can access the pointer. 

```c++
std::atomic<MyType *> atomic_ptr;
atomic_ptr.store(new MyType);
//...
MyType *old_ptr = atomic_ptr.swap(new MyType);
HazPtrRetire(old_ptr); // The old_ptr will be freed by default deleter -> delete old_ptr
```

- You can customize the deleter

```c++
HazPtrRetire(old_ptr, [](void *to_be_freed) {
    MyType *ptr = (MyType*)to_be_freed;
    ptr->CleanSomeThing();
   	delete old_ptr;
});
```

## Pros and Cons of My Implementation

- Pros:
  - Easy to Use
  - Fast

- Cons:
  - Too Simple
  - Needs to declare some global state and probably initialize it
  - Can only support one global state

## Other Implementation

- [folly::haz_ptr](https://github.com/facebook/folly)  by Facebook

## Other Related Techniques

- Epoch Based Reclaimation
- Atomic Reference Counting

