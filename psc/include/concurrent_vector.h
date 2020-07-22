#ifndef CONCURRENT_VECTOR_H
#define CONCURRENT_VECTOR_H

#include <mutex>
#include <vector>
#include "speed.h"
using namespace std;

template<typename T>
class concurrent_vector: public std::vector<T> {
private:
	std::mutex mylock;
public:
	concurrent_vector<T>() : vector<T>(){
	}
	concurrent_vector<T>(const concurrent_vector<T>& x) :
			vector<T>(x) {

	}
//	concurrent_vector<T>(concurrent_vector<T>&& x) :
//			vector<T>(x) {
//	}
//	concurrent_vector<T> (concurrent_vector<T>&& x, const std::allocator<T>& alloc): vector<T>(x,alloc){}

	concurrent_vector& operator=(const concurrent_vector<T>&x) {
		vector<T>::operator=(x);
		return *this;
	}
	void push_back(T val) {
		mylock.lock();
		vector < T > ::push_back(val);
		mylock.unlock();
	}
};
#endif
