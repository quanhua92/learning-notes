#include <iostream>
#include <type_traits>
template<typename T1, typename T2, 
	typename RT=std::decay_t<decltype(true ? T1() : T2())>>
RT max (T1 a, T2 b)
{
	return b < a ? a : b;
}

//void main() {
void main_default1() {
	std::cout << ::max(40, 10.2) << std::endl;
	std::cout << ::max(5.5, 10.2) << std::endl;
	std::cout << ::max(5.5, 2) << std::endl;

	system("pause");
}
