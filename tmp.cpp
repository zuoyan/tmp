/**
 * @file   tmp.cpp
 * @author Changsheng Jiang <jiangzuoyan@gmail.com>
 * @date   Fri Nov  9 23:05:42 2012
 *
 * @brief test tmp in compile time
 *
 *
 */
#include <cstdint>

#include "tmp.hpp"
#include <boost/mpl/placeholders.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/list.hpp>
#include <boost/mpl/vector_c.hpp>
#include <boost/mpl/plus.hpp>
#include <boost/mpl/less.hpp>
#include <boost/mpl/sizeof.hpp>

#include <tuple>

template <class A, class B>
void assert_type() {
  static_assert(std::is_same<A, B>::value, "...");
}

using namespace tmp;
using boost::mpl::_1;
using boost::mpl::_2;
using boost::mpl::_3;
using boost::mpl::_4;
using boost::mpl::_5;
using boost::mpl::_6;

template <class A>
struct is_odd : constant<bool, (A::value % 2)> {};

template <class A>
struct is_even : constant<bool, (A::value % 2 == 0)> {};

template <class A>
struct to_pair : identity< std::pair<A, A> > {};

template <class A>
struct square : int_<A::value * A::value> {};

namespace details {

template<class Pred, class S>
struct foo : boost::mpl::plus<
  typename square<
    typename len<
      typename filter<Pred, S>::type
      >::type>::type,
  int_<5> > {};

template<class Pred, class S>
struct compound_foo : compound<
  typename filter<Pred, S>::type,
  len<boost::mpl::_1>,
  square<boost::mpl::_1>,
  boost::mpl::plus<int_<5>, boost::mpl::_1> > {};

void test1() {
  typedef boost::mpl::less<boost::mpl::_1, int_<5> > pred;
  typedef range_c<1, 10>::type list;
  assert_type< int_<foo<pred, list>::type::value>,
               int_<21> >();
  assert_type< int_<compound_foo<pred, list>::type::value>,
               int_<21> >();
}

} // namespace details


int main(int argc, char *argv[]) {

  assert_type<
    reverse< vector<int, float, double> >::type,
    vector<double, float, int> >();

  assert_type<
    reverse< vector_c<int, 1, 2, 3, 4> >::type,
    vector_c<int, 4, 3, 2, 1> >();

  assert_type<
    nth_c<1, vector<int, float, double> >::type,
    float >();

  assert_type<
    nth_c<1, vector<int, void, double> >::type,
    void >();

  static_assert(len<vector_c<int, 0, 1, 2, 3> >::value == 4, "...");
  static_assert(len<vector<int, float, double> >::value == 3, "...");
  static_assert(len<vector<void, void, double> >::value == 3, "...");

  assert_type< cdr< vector_c<int, 0, 1, 2> >::type,
               vector_c<int, 1, 2> >();

  assert_type< car< vector_c<int, 0, 1, 2> >::type,
               constant<int, 0> >();

  assert_type< cdr< vector_c<int> >::type,
               vector_c<int> >();

  assert_type< cat< vector<int, float>, vector<int, char, double> >::type,
               vector<int, float, int, char, double> >();

  assert_type< cat< vector_c<int, 0, 1, 2>, vector_c<int, 0, 1, 3> >::type,
               vector_c<int, 0, 1, 2, 0, 1, 3> >();

  assert_type< map< boost::mpl::plus<_1, int_<1> >, vector_c<int, 0, 1, 2> >::type,
               vector<boost::mpl::integral_c<int, 1>,
                      boost::mpl::integral_c<int, 2>,
                      boost::mpl::integral_c<int, 3>
                      > >();

  assert_type< map< to_pair<_1>, vector<int, float, double> >::type,
               vector<std::pair<int, int>,
                      std::pair<float, float>,
                      std::pair<double, double> > >();

  assert_type< take< int_<2>, vector<int, float, char, double> >::type,
               vector<int, float> >();

  static_assert(boost::mpl::apply< boost::mpl::less<_1, int_<2> >, int_<1> >::type::value, "...");
  static_assert(not boost::mpl::apply< boost::mpl::less<_1, int_<1> >, int_<2> >::type::value, "...");
  static_assert(boost::mpl::apply<
                  boost::mpl::not_< boost::mpl::less<_1, int_<1> > >,
                  int_<2> >::type::value, "...");

  assert_type< sort< boost::mpl::less<_1, _2>,
                     vector_c<int, 3, 2, 1, 10, 4, 11, 30, 70, 20> >::type,
               vector_c<int, 1, 2, 3, 4, 10, 11, 20, 30, 70> >();

  assert_type< sort< boost::mpl::less<boost::mpl::sizeof_<_1>, boost::mpl::sizeof_<_2> >,
                     vector<std::tuple<int, int>, int, char, std::tuple<int, int, int> > >::type,
               vector<char, int, std::tuple<int, int>, std::tuple<int, int, int> > >();

  assert_type< int_< lower_bound< boost::mpl::less<_1, _2>, int_<0>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<0> >();
  assert_type< int_< lower_bound< boost::mpl::less<_1, _2>, int_<1>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<0> >();
  assert_type< int_< lower_bound< boost::mpl::less<_1, _2>, int_<2>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<1> >();
  assert_type< int_< lower_bound< boost::mpl::less<_1, _2>, int_<4>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<3> >();
  assert_type< int_< lower_bound< boost::mpl::less<_1, _2>, int_<5>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<6> >();
  assert_type< int_< lower_bound< boost::mpl::less<_1, _2>, int_<10>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<6> >();
  assert_type< int_< lower_bound< boost::mpl::less<_1, _2>, int_<11>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<8> >();

  assert_type< int_< upper_bound< boost::mpl::less<_1, _2>, int_<0>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<0> >();
  assert_type< int_< upper_bound< boost::mpl::less<_1, _2>, int_<1>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<1> >();
  assert_type< int_< upper_bound< boost::mpl::less<_1, _2>, int_<2>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<2> >();
  assert_type< int_< upper_bound< boost::mpl::less<_1, _2>, int_<4>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<6> >();
  assert_type< int_< upper_bound< boost::mpl::less<_1, _2>, int_<5>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<6> >();
  assert_type< int_< upper_bound< boost::mpl::less<_1, _2>, int_<10>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<8> >();
  assert_type< int_< upper_bound< boost::mpl::less<_1, _2>, int_<11>,
                                  vector_c<int, 1, 2, 3, 4, 4, 4, 10, 10> >::value>,
               int_<8> >();

  static_assert(all_of<vector_c<int, 1, 3, 5, 7>, is_odd<_1> >::value, "...");
  static_assert(any_of<vector_c<int, 1, 3, 5, 7>, is_odd<_1> >::value, "...");
  static_assert(not none_of<vector_c<int, 1, 3, 5, 7>, is_odd<_1> >::value, "...");

  static_assert(not all_of<vector_c<int, 1, 2, 5, 7>, is_odd<_1> >::value, "...");
  static_assert(any_of<vector_c<int, 1, 2, 5, 7>, is_odd<_1> >::value, "...");
  static_assert(not none_of<vector_c<int, 1, 2, 5, 7>, is_odd<_1> >::value, "...");
  static_assert(none_of<vector_c<int, 2, 4, 6>, is_odd<_1> >::value, "...");
  static_assert(count_if<vector_c<int, 2, 1, 3, 5, 4, 6>, is_odd<_1> >::value == 3, "...");

  static_assert(find_if<vector_c<int, 2, 4, 3, 5, 4, 6>, is_odd<_1> >::type::value == 2, "...");
  static_assert(find<vector_c<int, 2, 4, 3, 5, 4, 6>, constant<int, 3> >::type::value == 2, "...");

  assert_type< seq_from_boost_mpl<boost::mpl::vector<int, float> >::type,
               vector<int, float> >();

  assert_type< seq_from_boost_mpl<boost::mpl::vector<int> >::type,
               vector<int> >();

  assert_type< seq_from_boost_mpl<boost::mpl::vector<> >::type,
               vector<> >();

  assert_type< seq_from_boost_mpl<boost::mpl::list<int, float> >::type,
               vector<int, float> >();

  assert_type< seq_to_boost_mpl<vector<int, float>, boost::mpl::vector<> >::type,
               boost::mpl::vector<int, float> >();

  assert_type< seq_to_boost_mpl<vector<int, float, double, char>, boost::mpl::list<> >::type,
               boost::mpl::list<int, float, double,char> >();

  assert_type< seq_to_boost_mpl<vector<int, float, double, char>, boost::mpl::list<> >::type,
               boost::mpl::list<int, float, double,char> >();

  return 0;
}
