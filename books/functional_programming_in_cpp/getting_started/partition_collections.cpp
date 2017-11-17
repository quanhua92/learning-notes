#include <vector>
#include <iostream>
#include <iterator>
#include <algorithm>

bool is_selected(int value) {
	return (value == 3) || (value == 5) || (value == 8);
}

void main_partition_collections() {
	//void main() {
	std::vector<int> scores = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
	std::cout << "Original Vector " << std::endl;
	for (auto e : scores) std::cout << e << " ";
	std::cout << std::endl;

	//auto it = std::partition(scores.begin(), scores.end(), is_selected);
	auto it = std::stable_partition(scores.begin(), scores.end(), is_selected);
	std::cout << "Partitioned values:" << std::endl;
	std::copy(std::begin(scores), it, std::ostream_iterator<int>(std::cout, " "));
	std::cout << " * ";
	std::copy(it, std::end(scores), std::ostream_iterator<int>(std::cout, " "));

	system("pause");
}



