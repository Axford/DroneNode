#pragma once
#include <vector>
namespace IvanLinkedList {template<class T>struct LinkedList:std::vector<T> {void add(T value){this->push_back(value);}};}
