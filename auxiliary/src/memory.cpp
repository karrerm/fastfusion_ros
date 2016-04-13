/*
 * memory.cpp
 *
 *  Created on: Feb 27, 2013
 *      Author: steinbrf
 */

#include "auxiliary/memory.hpp"
#include <unistd.h>
#include <fstream>
#include <string>

#ifdef __APPLE__
#include <mach/mach.h>
#include <mach/vm_map.h>
#include <sys/sysctl.h>
#endif


long long getTotalSystemMemory()
{
#ifdef __APPLE__
    long long phys_mem_size = 0;
    size_t len = sizeof(phys_mem_size);
    int ret = -1;

    /* Note hw.memsize is in bytes, so no need to multiply by page size. */
    ret = sysctlbyname("hw.memsize", &phys_mem_size, &len, NULL, 0);
    if (ret == -1) {
      phys_mem_size = 0;
      return 0;
    }
    return phys_mem_size;
#else
    long long pages = sysconf(_SC_PHYS_PAGES);
    long long page_size = sysconf(_SC_PAGE_SIZE);
    return pages * page_size;
#endif
}

long long getAvailableSystemMemory()
{
#ifdef __APPLE__
    // TODO(ffurrer): check if there is no cleaner way of doing this!
    long long page_size, page_free_count;
    size_t len_page_free_count = sizeof(page_free_count);
    size_t len_page_size = sizeof(page_size);
    int ret2 = sysctlbyname("vm.page_free_count", &page_free_count, &len_page_free_count, NULL, 0);
    int ret3 = sysctlbyname("vm.pagesize", &page_size, &len_page_size, NULL, 0);
    return page_free_count * page_size;
#else
    long long pages = sysconf(_SC_AVPHYS_PAGES);
    long long page_size = sysconf(_SC_PAGE_SIZE);
    return pages * page_size;
#endif
}

ProcessMemoryStats getProcessMemory(){
	std::fstream file("/proc/self/statm");
	ProcessMemoryStats result;
	file >> result.size;
	file >> result.resident;
	file >> result.shared;
	file >> result.trs;
	file >> result.drs;
	file >> result.lrs;
	file >> result.dt;
	result.pageSize = sysconf(_SC_PAGE_SIZE);
	return result;
}
