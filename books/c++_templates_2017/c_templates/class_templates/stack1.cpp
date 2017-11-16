#pragma once
#include <cassert>
#include <iostream>
#include <string>
#include <vector>

template<typename T>
class Stack {
private:
	std::vector<T> elems; // elements
public:
	void push(T const& elem); // push element
	void pop();
	T const& top() const; // return top element
	bool empty() const {
		return elems.empty();
	}

	void printOn(std::ostream& strm) const;
};

template<typename T>
void Stack<T>::push(T const& elem) {
	elems.push_back(elem); // append copy of passed elem
}

template<typename T>
void Stack<T>::pop() {
	assert(!elems.empty());
	elems.pop_back();
}

template<typename T>
T const& Stack<T>::top() const {
	assert(!elems.empty());
	return elems.back(); // return copy of last element
}

template<typename T>
void Stack<T>::printOn(std::ostream& strm) const{
	for (T const& elem : elems) {
		strm << elem << " "; // call << for each element
	}
}
//void main_stack1() {
void main(){
	Stack<int> intStack;
	Stack<std::string> stringStack;

	intStack.push(7);
	std::cout << intStack.top() << std::endl;

	stringStack.push("hello");
	stringStack.push("world");
	std::cout << stringStack.top() << std::endl;

	stringStack.printOn(std::cout);

	stringStack.pop();
	system("pause");
}