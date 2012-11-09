/**
 * @file   tmp.hpp
 * @author Changsheng Jiang <jiangzuoyan@gmail.com>
 * @date   Fri Nov  9 23:05:57 2012
 *
 * @brief Template Meta-Programming
 *
 * type list in lisp style, with c++0x
 *
 * this is tested with g++-4.5, g++-4.6 and g++-4.7
 *
 */
#ifndef FILE_1c02f621_dba2_47e6_9649_6ef16cdeeae1_H
#define FILE_1c02f621_dba2_47e6_9649_6ef16cdeeae1_H

#include <type_traits>

#include <boost/mpl/if.hpp>
#include <boost/mpl/eval_if.hpp>
#include <boost/mpl/apply.hpp>
#include <boost/mpl/not.hpp>
#include <boost/mpl/placeholders.hpp>
#include <boost/mpl/bind.hpp>
#include <boost/mpl/same_as.hpp>
#include <boost/mpl/size.hpp>
#include <boost/mpl/at.hpp>
#include <boost/config.hpp>

namespace tmp {

template <class ...T>
struct vector {
  typedef vector type;
};

template <class H, class ...T>
struct vector<H, T...> {
  typedef vector type;
};

template <class T, T ...v>
struct vector_c {
  typedef vector_c type;
};

template <class T, T v>
struct constant {
  static const T value = v;
  typedef T value_type;
  typedef constant type;
  typedef typename boost::mpl::if_<
    std::is_integral<T>,
    boost::mpl::integral_c_tag,
    void>::type tag;
};

template <class T, int depth=1>
struct identity {
  typedef identity<T, depth-1> type;
};

template <class T>
struct identity<T, 1> {
  typedef T type;
};

template <class T, int depth=1>
struct untype : untype<T, depth-1>::type {};

template <class T>
struct untype<T, 0> : identity<T> {};

#if defined(BOOST_NO_CXX11_TEMPLATE_ALIASES) || defined(BOOST_NO_TEMPLATE_ALIASES)
template <int v>
struct int_ : constant<int, v> {};
#else
template <int V>
using int_ = constant<int, V>;
#endif

template <class ...More>
struct prog;

template <class Cond, class Expr, class ...More>
struct prog<Cond, Expr, More...> : boost::mpl::if_<
  Cond,
  identity<Expr>,
  prog<More...>
  >::type {};

template <class Expr>
struct prog<Expr> : identity<Expr> {};

template <class ...More>
struct eval_prog : prog<More...>::type {};

template <class Pred, class Trans, class State,
          bool End=boost::mpl::apply<Pred, State>::type::value>
struct while_ : while_<Pred, Trans,
                       typename boost::mpl::apply<Trans, State>::type> {};

template <class Pred, class Trans, class State>
struct while_<Pred, Trans, State, false> : identity<State> {};

template <class ...T>
struct all : std::true_type {};

template <class H, class ...T>
struct all<H, T...> : boost::mpl::eval_if<H, all<T...>, std::false_type> {};

template <class ...T>
struct any : std::false_type {};

template <class H, class ...T>
struct any<H, T...> : boost::mpl::eval_if<H, std::true_type, any<T...> > {};

template <class ...T>
struct none : std::true_type {};

template <class H, class ...T>
struct none<H, T...> : boost::mpl::eval_if<H, std::false_type, none<T...> > {};

template <class ...S>
struct cat;

template <class S>
struct cat<S> : identity<S> {};

template <class S, class ...More>
struct cat<S, More...> : cat<S, typename cat<More...>::type> {};

template <template <class...> class Seq, class ...T1,
          template <class...> class S2, class ...T2>
struct cat<Seq<T1...>, S2<T2...> > : Seq<T1..., T2...> {};

// value vector
template <template <class ST, ST...> class Seq, class T, T ...v1,
          template <class ST2, ST2...> class S2, class T2, T2 ...v2>
struct cat<Seq<T, v1...>, S2<T2, v2...> > : identity< Seq<T, v1..., v2...> > {};

template <class T, class Seq>
struct cons;

template <class T, template <class...> class Seq, class ...TS>
struct cons<T, Seq<TS...> > : Seq<T, TS...> { };

template <class TA, template <class ST, ST...> class Seq, class T, T ...V>
struct cons<TA, Seq<T, V...> > : Seq<T, TA::value, V...> { };

template <class T, T v, class Seq>
struct cons_c;

template <class T, T v, T ...SV, template <class ST, ST...> class Seq>
struct cons_c<T, v, Seq<T, SV...> > {
  typedef Seq<T, v, SV...> type;
};

template <class Seq, class ...T>
struct append;

template <template <class ...> class Seq, class ...ST, class ...T>
struct append<Seq<ST...>, T...> {
  typedef Seq<ST..., T...> type;
};

template <template <class ST_, ST_ ...> class Seq, class ST, ST ...SV, class ...T>
struct append<Seq<ST, SV...>, T...> {
  typedef Seq<ST, SV..., T::value...> type;
};

template <class Seq, class ...T>
struct push_back : append<Seq, T...> {};

template <class Seq, class T>
struct push_front : cons<T, Seq> {};

template <class Seq>
struct len : int_<0> {};

template <template <class...> class Seq, class ...T>
struct len< Seq<T...> > : int_<sizeof...(T)> { };

template <template <class T_, T_...> class Seq, class T, T ...V>
struct len< Seq<T, V...> > : int_<sizeof...(V)> { };

static_assert(len< vector<> >::value == 0, "...");
static_assert(len< vector<int, float> >::value == 2, "...");

template <class Seq>
struct car;

template <template <class ...> class Seq, class H, class ...T>
struct car< Seq<H, T...> > : identity<H> {};

template <template <class T_, T_...> class Seq, class T, T H, T ...V>
struct car< Seq<T, H, V...> > : constant<T, H> {};

template <class Seq>
struct cdr : identity<Seq> {};

template <template <class ...> class Seq, class H, class ...T>
struct cdr< Seq<H, T...> > : identity< Seq<T...> > {};

template <template <class T_, T_...> class Seq, class T, T H, T ...V>
struct cdr< Seq<T, H, V...> > : identity< Seq<T, V...> > {};

template <class Seq>
struct pop_front : cdr<Seq> {};

template <class Seq>
struct pop_back : identity<Seq> {};

template <template <class ...> class Seq, class H, class ...T>
struct pop_back< Seq<T..., H> > : identity< Seq<T...> > {};

template <template <class T_, T_...> class Seq, class T, T H, T ...V>
struct pop_back< Seq<T, V..., H> > : identity< Seq<T, V...> > {};

template <class S>
struct cadr : car<typename cdr<S>::type> {};

template <class S>
struct cdar : cdr<typename car<S>::type> {};

template <int N, class Seq>
struct nth_c : nth_c<N - 1, typename cdr<Seq>::type> {};

template <class Seq>
struct nth_c<0, Seq> : car<Seq> {};

template <class N, class Seq>
struct nth : nth_c<N::value, Seq> {};

template <class Seq, int L=len<Seq>::value>
struct reverse_impl : append<typename reverse_impl<typename cdr<Seq>::type>::type,
                             typename car<Seq>::type> {};

template <class Seq>
struct reverse_impl<Seq, 0> : identity<Seq> {};

template <class Seq>
struct reverse : reverse_impl<Seq> {};

template <class Map, class S>
struct map;

template <class Map, template <class ...> class S, class ...T>
struct map<Map, S<T...> > {
  typedef S<typename boost::mpl::apply<Map, T>::type... > type;
};

template <class Map, template <class T_, T_ ...> class S, class T, T ...V>
struct map<Map, S<T, V...> > {
  typedef vector<typename boost::mpl::apply<Map, constant<T, V> >::type... > type;
};

template <class S, class ...T>
struct vrebind;

template <template <class...> class S, class ...Old, class ...T>
struct vrebind< S<Old...>, T...> {
  typedef S<T...> type;
};

template <template <class T_, T_...> class S, class TO, TO...OV, class TN, TN ...NV>
struct vrebind< S<TO, OV...>, constant<TN, NV>...> {
  typedef S<TN, NV...> type;
};

template <class S, class ...T>
struct eval_vrebind : vrebind<typename S::type, typename T::type...> {};

template <class S, class Args>
struct rebind;

template <class S, template <class...> class AS, class ...AT>
struct rebind<S, AS<AT...> > : vrebind<S, AT...> {};

template <class S, template <class T_, T_...> class AS, class AT, AT ...AV>
struct rebind<S, AS<AT, AV...> > : vrebind<S, constant<AT, AV>...> {};

template <class S, class Args>
struct eval_rebind : rebind<typename S::type, typename Args::type> {};

template <class F, class ...A>
struct curry;

template <template <class ...> class F, class ...Old, class ...A>
struct curry<F<Old...>, A...> {
  template <class ...T>
  struct apply {
    typedef F<A..., T...> type;
  };
};

template <class Init, class ...Statement>
struct compound : identity<Init> {};

template <class Init, class Statement, class ...More>
struct compound<Init, Statement, More...> :
      compound<typename boost::mpl::apply<Statement, Init>::type, More...> {};

template <class S>
struct empty_like;

template <template <class ...> class S, class ...T>
struct empty_like< S<T...> > : identity< S<> > {};

template <template <class T_, T_ ...> class S, class T, T ...V>
struct empty_like< S<T, V...> > : identity< S<T> > {};

template <int N, class S>
struct take_c : cons<typename car<S>::type,
                     typename take_c<N - 1, typename cdr<S>::type>::type> {};

template <class S>
struct take_c<0, S> : empty_like<S> {};

template <class N, class S>
struct take : take_c<N::value, S> {};

template <int N, class S>
struct drop_c : drop_c<N - 1, typename cdr<S>::type> {};

template <class S>
struct drop_c<0, S> : identity<S> {};

template <class N, class S>
struct drop : drop_c<N::value, S> {};

template <int N, class S>
struct split_c {
  typedef vector< typename take_c<N, S>::type,
                  typename drop_c<N, S>::type > type;
};

template <class N, class S>
struct split : split_c<N::value, S> {};

template <int S, int E, class Seq>
struct slice_c : drop_c<S, typename take_c<E, Seq>::type> {};

template <class S, class E, class L>
struct slice : slice_c<S::value, E::value, L> {};

template <int N, class S>
struct rotate_c {
 private:
  typedef typename split_c<N % len<S>::value, S>::type parts;
 public:
  typedef typename cat<
   typename empty_like<S>::type,
   typename nth_c<0, parts>::type,
   typename nth_c<1, parts>::type>::type type;
};

template <int N, class S>
struct remove_c : cons<
  typename car<S>::type,
  typename remove_c<N - 1, typename cdr<S>::type>::type > { };

template <class S>
struct remove_c<0, S> : cdr<S> {};

template <class N, class S>
struct remove : remove_c<N::value, S> {};

template <int N, class V, class S>
struct insert_c : cons<
  typename car<S>::type,
  typename insert_c<N - 1, V, typename cdr<S>::type>::type> {};

template <class V, class S>
struct insert_c<0, V, S> : cons<V, S> {};

template <class N, class V, class S>
struct insert : insert_c<N::value, V, S> {};

template <int S, int E>
struct range_c : cons<constant<int, S>,
                      typename range_c<S + 1, E>::type> {};

template <int S>
struct range_c<S, S> {
  typedef vector_c<int> type;
};

template <class S, class E>
struct range : range_c<S::value, E::value> { };

template <class Func, class Init, class S, int L=len<S>::value>
struct fold_impl : boost::mpl::apply<
  Func,
  typename car<S>::type,
  typename fold_impl<Func, Init, typename cdr<S>::type>::type
  > {};

template <class Func, class Init, class S>
struct fold_impl<Func, Init, S, 0> : identity<Init> {};

template <class Func, class Init, class S>
struct fold : fold_impl<Func, Init, S> {};

template <class Map, class Fold, class Init, class S>
struct map_fold : fold<Fold, Init, typename map<Map, S>::type> {};

template <class Pred, class S, int L=len<S>::value>
struct filter_impl {
 private:
  typedef typename filter_impl<Pred, typename cdr<S>::type>::type rest;
 public:
  typedef typename boost::mpl::if_<
   typename boost::mpl::apply<Pred, typename car<S>::type>::type,
   typename cons<typename car<S>::type, rest>::type,
   rest>::type type;
};

template <class Pred, class S>
struct filter_impl<Pred, S, 0> : identity<S> {};

template <class Pred, class S>
struct filter : filter_impl<Pred, S> {};

template <class Pred, class S>
struct filter_out : filter<boost::mpl::not_<Pred>, S> {};

template <class Less, class V, class L>
struct partition {
  template <class T>
  struct less_v : boost::mpl::apply<Less, T, V> {};

  typedef vector<
    typename filter< less_v<boost::mpl::_1>, L>::type,
    typename filter_out<less_v<boost::mpl::_1>, L>::type
    > type;
};

template <class Less, class S, int L=len<S>::value>
struct sort_impl {
  typedef typename car<S>::type H;
  typedef typename partition<Less, H, typename cdr<S>::type >::type parts;
  typedef typename nth_c<0, parts>::type lt_part;
  typedef typename nth_c<1, parts>::type ge_part;
 public:
  typedef typename cat<
   typename sort_impl<Less, lt_part>::type,
   typename cons<H, typename sort_impl<Less, ge_part>::type>::type
  >::type type;
};

template <class Less, class S>
struct sort_impl<Less, S, 0> : identity<S> {};

template <class Less, class S>
struct sort_impl<Less, S, 1> : identity<S> {};

template <class Less, class S>
struct sort : sort_impl<Less, S> {};

template <class Less, class V, class S, int SI=0, int EI=len<S>::value>
struct lower_bound : boost::mpl::eval_if<
  typename boost::mpl::apply<
    Less,
    typename nth_c<(SI + (EI - SI) / 2), S>::type, V>::type,
  lower_bound<Less, V, S, SI + (EI - SI) / 2 + 1, EI>,
  lower_bound<Less, V, S, SI, SI + (EI - SI) / 2> >::type {};

template <class Less, class V, class S, int SI>
struct lower_bound<Less, V, S, SI, SI> : int_<SI> {};

template <class Less, class V, class S, int SI=0, int EI=len<S>::value>
struct upper_bound : boost::mpl::eval_if<
  typename boost::mpl::apply<
    boost::mpl::not_<Less>,
    V, typename nth_c<(SI + (EI - SI) / 2), S>::type>::type,
  upper_bound<Less, V, S, SI + (EI - SI) / 2 + 1, EI>,
  upper_bound<Less, V, S, SI, SI + (EI - SI) / 2 > >::type {};

template <class Less, class V, class S, int SI>
struct upper_bound<Less, V, S, SI, SI> : int_<SI> {};

template <class S, class Pred>
struct all_of : rebind<all<>, typename map<Pred, S>::type>::type::type {};

template <class S, class Pred>
struct any_of : rebind<any<>, typename map<Pred, S>::type>::type::type {};

template <class S, class Pred>
struct none_of : rebind<none<>, typename map<Pred, S>::type>::type::type {};

#if defined(BOOST_NO_CXX11_CONSTEXPR) || defined(BOOST_NO_CONSTEXPR)
template <class ...A>
struct sum : int_<0> {};

template <class H, class ...T>
struct sum<H, T...> : int_<H::value + sum<T...>::value> {};
#else
inline constexpr int sum_c() {
  return 0;
}

template <class ...T>
inline constexpr int sum_c(int a, T ...t) {
  return a + sum_c(t...);
}

template <class ...A>
struct sum : int_<sum_c(A::value...)> {};
#endif

template <class S, class Pred>
struct count_if : rebind<sum<>, typename map<Pred, S>::type>::type::type {};

template <class A, class B,
          int LA=len<A>::value, int LB=len<B>::value>
struct seq_equal : std::false_type {};

template <class A, class B, int L>
struct seq_equal<A, B, L, L> {
  typedef typename boost::mpl::eval_if<
    std::is_same<typename car<A>::type, typename car<B>::type>,
    seq_equal<typename cdr<A>::type, typename cdr<B>::type>,
    std::false_type>::type type;
};

template <class A, class B>
struct seq_equal<A, B, 0, 0> : std::true_type {};

template <class Seq, class Pred, int L=len<Seq>::value>
struct find_if_impl {
  struct find_cdr {
    typedef int_<
      find_if_impl<typename cdr<Seq>::type, Pred>::type::value + 1> type;
  };

  typedef typename boost::mpl::eval_if<
    typename boost::mpl::apply<Pred, typename car<Seq>::type>::type,
    identity< int_<0> >,
    find_cdr
    >::type type;
};

template <class Seq, class Pred>
struct find_if_impl<Seq, Pred, 0> : int_<0> {};

template <class Seq, class Pred>
struct find_if : find_if_impl<Seq, Pred> {};

template <class Seq, class V>
struct find : find_if<Seq, boost::mpl::same_as<V> > {};

template <class S>
struct seq_from_boost_mpl :
      map<boost::mpl::at<S, boost::mpl::_1>,
          typename range_c<0, boost::mpl::size<S>::value>::type > { };

// D = boost::mpl::vector<> or list ...
template <class S, class D>
struct seq_to_boost_mpl : rebind<D, S> {};

} // namespace tmp
#endif
