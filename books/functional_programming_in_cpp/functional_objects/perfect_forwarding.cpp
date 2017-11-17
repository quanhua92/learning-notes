#include <utility>

template <typename Object, typename Function>
decltype(auto) call_on_object(Object&& object, Function function) {
	return function(std::forward<Object>(object));
}
//Object && ----> double reference (forwarding reference)
//			- accept both const and non-const objects and temporaries.