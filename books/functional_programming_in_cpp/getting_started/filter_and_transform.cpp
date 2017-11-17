#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

struct Person {
	bool is_female;
	std::string name;
};
bool is_female(const Person &person) {
	return person.is_female;
}

bool is_not_female(const Person &person) {
	return !person.is_female;
}

std::string name(const Person &person) {
	return person.name;
}

std::vector<Person> filter_by_remove(std::vector<Person> people) {
	people.erase(std::remove_if(people.begin(), people.end(), is_not_female), people.end());
	return people;
}

std::vector<Person> filter_by_copy(std::vector<Person> people) {
	std::vector<Person> females;
	std::copy_if(people.cbegin(), people.cend(), std::back_inserter(females), is_female);
	return females;
}

//void main_filter_and_transform() {
void main() {
	Person a;
	a.is_female = true;
	a.name = "Marry";
	Person b;
	b.is_female = true;
	b.name = "Rose";
	Person c;
	c.is_female = false;
	c.name = "John";
	std::vector<Person> people = { a, b, c };

	std::cout << "Original vector: " << std::endl;
	for (auto elem : people) std::cout << elem.name << " ";
	std::cout << std::endl;

	std::cout << "Filter by remove: " << std::endl;
	for (auto elem : filter_by_remove(people)) std::cout << elem.name << " ";
	std::cout << std::endl;

	std::cout << "Filter by copy: " << std::endl;
	for (auto elem : filter_by_copy(people)) std::cout << elem.name << " ";
	std::cout << std::endl;

	std::cout << "Get the names by transform: " << std::endl;
	std::vector<Person> females = filter_by_copy(people);
	std::vector<std::string> names(females.size());
	std::transform(females.cbegin(), females.cend(), names.begin(), name);

	for (auto elem : names) std::cout << elem << " ";
	std::cout << std::endl;
	
	// PROBLEM: UNNECESSARY MEMORY ALLOCATIONS
	// SOLUTION: USE RANGES LIBRARY
	system("pause");

}