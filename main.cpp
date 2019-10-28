#include <iostream>
#include "haz_ptr.h"

int main() {
    std::cout << "Hello, World!" << std::endl;
    int *ptr = new int(1);
    HazPtrRetire(ptr);

    return 0;
}
