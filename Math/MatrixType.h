#pragma once

template <class Type>
class CSymMat {
public:
	CSymMat(void) {}
	CSymMat(int s) { Resize(s); }
	CSymMat(int s, const Type &v) { Resize(s, v); }
	std::vector<Type> m;
	unsigned int n;
	bool IsEmpty(void) { return m.empty(); }
	void Resize(int s) { n = s; m.resize(n*(n + 1) / 2); }
	void Resize(int s, const Type &v) { n = s; m.resize(n*(n + 1) / 2, v); }
	void Set(int i, int j, const Type &v) {
		if (i > j) { std::swap(i, j); }
		m[i*(i + 1) / 2 + j] = v;
	}
	Type Get(int i, int j) {
		if (i > j) { std::swap(i, j); }
		return m[i*(i + 1) / 2 + j];
	}
	const Type& Get(int i, int j) const {
		if (i > j) { std::swap(i, j); }
		return m[i*(i + 1) / 2 + j];
	}
};

template <class Type>
class CDistMat : public CSymMat < Type > {
public:
	CDistMat(void) {}
	CDistMat(int s) { Resize(s); }
	CDistMat(int s, const Type &v) { Resize(s, v); }
	void Resize(int s) { n = s; m.resize(n*(n - 1) / 2); }
	void Resize(int s, const Type &v) { n = s; m.resize(n*(n - 1) / 2, v); }
	void Set(int i, int j, const Type &v) {
		if (i == j) { return; }
		if (i > j) { std::swap(i, j); }
		m[i*(2 * n - i - 1) / 2 + j - i - 1] = v;
	}
	Type Get(int i, int j) {
		if (i == j) { return 0; }
		else if (i > j) { std::swap(i, j); }
		return m[i*(2 * n - i - 1) / 2 + j - i - 1];
	}
	const Type& Get(int i, int j) const {
		if (i == j) { return 0; }
		else if (i > j) { std::swap(i, j); }
		return m[i*(2 * n - i - 1) / 2 + j - i - 1];
	}
};