# /* Copyright (C) 2001
#  * Housemarque Oy
#  * http://www.housemarque.com
#  *
#  * Distributed under the Boost Software License, Version 1.0. (See
#  * accompanying file LICENSE_1_0.txt or copy at
#  * http://www.boost.org/LICENSE_1_0.txt)
#  */
#
# /* Revised by Paul Mensonides (2002) */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef FARM_PP_PREPROCESSOR_LIST_FOR_EACH_PRODUCT_HPP
# define FARM_PP_PREPROCESSOR_LIST_FOR_EACH_PRODUCT_HPP
#
# include <farm_pp/preprocessor/config/config.hpp>
# include <farm_pp/preprocessor/control/if.hpp>
# include <farm_pp/preprocessor/facilities/overload.hpp>
# include <farm_pp/preprocessor/list/adt.hpp>
# include <farm_pp/preprocessor/list/to_tuple.hpp>
# include <farm_pp/preprocessor/repetition/for.hpp>
# include <farm_pp/preprocessor/tuple/elem.hpp>
# include <farm_pp/preprocessor/tuple/to_list.hpp>
# include <farm_pp/preprocessor/tuple/rem.hpp>
# include <farm_pp/preprocessor/tuple/reverse.hpp>
# if FARM_PP_VARIADICS_MSVC
# include <farm_pp/preprocessor/cat.hpp>
# include <farm_pp/preprocessor/facilities/empty.hpp>
# endif
#
# /* FARM_PP_LIST_FOR_EACH_PRODUCT */
#
# define FARM_PP_LIST_FOR_EACH_PRODUCT_OV_2(macro, size, tuple) FARM_PP_LIST_FOR_EACH_PRODUCT_E(FARM_PP_FOR, macro, size, FARM_PP_TUPLE_TO_LIST(size, tuple))
# define FARM_PP_LIST_FOR_EACH_PRODUCT_OV_1(macro, tuple) FARM_PP_LIST_FOR_EACH_PRODUCT_E(FARM_PP_FOR, macro, FARM_PP_TUPLE_SIZE(tuple), FARM_PP_TUPLE_TO_LIST(tuple))
#
# if ~FARM_PP_CONFIG_FLAGS() & FARM_PP_CONFIG_EDG()
#    if FARM_PP_VARIADICS_MSVC
#        define FARM_PP_LIST_FOR_EACH_PRODUCT(macro, ...) FARM_PP_CAT(FARM_PP_OVERLOAD(FARM_PP_LIST_FOR_EACH_PRODUCT_OV_,__VA_ARGS__)(macro,__VA_ARGS__),FARM_PP_EMPTY())
#    else
#        define FARM_PP_LIST_FOR_EACH_PRODUCT(macro, ...) FARM_PP_OVERLOAD(FARM_PP_LIST_FOR_EACH_PRODUCT_OV_,__VA_ARGS__)(macro,__VA_ARGS__)
#    endif
# else
#    define FARM_PP_LIST_FOR_EACH_PRODUCT(macro, ...) FARM_PP_LIST_FOR_EACH_PRODUCT_Q(macro, __VA_ARGS__)
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_Q(macro, ...) FARM_PP_OVERLOAD(FARM_PP_LIST_FOR_EACH_PRODUCT_OV_,__VA_ARGS__)(macro,__VA_ARGS__)
# endif
#
# /* FARM_PP_LIST_FOR_EACH_PRODUCT_R */
#
# define FARM_PP_LIST_FOR_EACH_PRODUCT_R_OV_2(r, macro, size, tuple) FARM_PP_LIST_FOR_EACH_PRODUCT_E(FARM_PP_FOR ## r, macro, size, FARM_PP_TUPLE_TO_LIST(size, tuple))
# define FARM_PP_LIST_FOR_EACH_PRODUCT_R_OV_1(r, macro, tuple) FARM_PP_LIST_FOR_EACH_PRODUCT_E(FARM_PP_FOR ## r, macro, FARM_PP_TUPLE_SIZE(tuple), FARM_PP_TUPLE_TO_LIST(tuple))
#
# if ~FARM_PP_CONFIG_FLAGS() & FARM_PP_CONFIG_EDG()
#    if FARM_PP_VARIADICS_MSVC
#        define FARM_PP_LIST_FOR_EACH_PRODUCT_R(r, macro, ...) FARM_PP_CAT(FARM_PP_OVERLOAD(FARM_PP_LIST_FOR_EACH_PRODUCT_R_OV_,__VA_ARGS__)(r, macro,__VA_ARGS__),FARM_PP_EMPTY())
#    else
#        define FARM_PP_LIST_FOR_EACH_PRODUCT_R(r, macro, ...) FARM_PP_OVERLOAD(FARM_PP_LIST_FOR_EACH_PRODUCT_R_OV_,__VA_ARGS__)(r, macro,__VA_ARGS__)
#    endif
# else
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_R(r, macro, ...) FARM_PP_LIST_FOR_EACH_PRODUCT_R_Q(r, macro, __VA_ARGS__)
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_R_Q(r, macro, ...) FARM_PP_OVERLOAD(FARM_PP_LIST_FOR_EACH_PRODUCT_R_OV_,__VA_ARGS__)(r, macro,__VA_ARGS__)
# endif
#
# if ~FARM_PP_CONFIG_FLAGS() & FARM_PP_CONFIG_EDG()
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_E(impl, macro, size, lists) impl((FARM_PP_LIST_FIRST(lists), FARM_PP_LIST_REST(lists), FARM_PP_NIL, macro, size), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_0)
# else
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_E(impl, macro, size, lists) FARM_PP_LIST_FOR_EACH_PRODUCT_E_D(impl, macro, size, lists)
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_E_D(impl, macro, size, lists) impl((FARM_PP_LIST_FIRST(lists), FARM_PP_LIST_REST(lists), FARM_PP_NIL, macro, size), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_0)
# endif
#
# if FARM_PP_CONFIG_FLAGS() & FARM_PP_CONFIG_STRICT()
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_P(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_P_I data
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_P_I(a, b, res, macro, size) FARM_PP_LIST_IS_CONS(a)
# else
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_P(r, data) FARM_PP_LIST_IS_CONS(FARM_PP_TUPLE_ELEM(5, 0, data))
# endif
#
# if ~FARM_PP_CONFIG_FLAGS() & FARM_PP_CONFIG_MWCC()
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_O(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_O_I data
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_O_I(a, b, res, macro, size) (FARM_PP_LIST_REST(a), b, res, macro, size)
# else
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_O(r, data) (FARM_PP_LIST_REST(FARM_PP_TUPLE_ELEM(5, 0, data)), FARM_PP_TUPLE_ELEM(5, 1, data), FARM_PP_TUPLE_ELEM(5, 2, data), FARM_PP_TUPLE_ELEM(5, 3, data), FARM_PP_TUPLE_ELEM(5, 4, data))
# endif
#
# if ~FARM_PP_CONFIG_FLAGS() & FARM_PP_CONFIG_EDG()
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_I(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_I_I(r, FARM_PP_TUPLE_ELEM(5, 0, data), FARM_PP_TUPLE_ELEM(5, 1, data), FARM_PP_TUPLE_ELEM(5, 2, data), FARM_PP_TUPLE_ELEM(5, 3, data), FARM_PP_TUPLE_ELEM(5, 4, data))
# else
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_I(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_I_D(r, FARM_PP_TUPLE_REM_5 data)
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_I_D(r, data_e) FARM_PP_LIST_FOR_EACH_PRODUCT_I_I(r, data_e)
# endif
#
# define FARM_PP_LIST_FOR_EACH_PRODUCT_I_I(r, a, b, res, macro, size) FARM_PP_LIST_FOR_EACH_PRODUCT_I_II(r, macro, FARM_PP_LIST_TO_TUPLE_R(r, (FARM_PP_LIST_FIRST(a), res)), size)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_I_II(r, macro, args, size) FARM_PP_LIST_FOR_EACH_PRODUCT_I_III(r, macro, args, size)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_I_III(r, macro, args, size) macro(r, FARM_PP_TUPLE_REVERSE(size, args))
#
# define FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, i) FARM_PP_IF(FARM_PP_LIST_IS_CONS(FARM_PP_TUPLE_ELEM(5, 1, data)), FARM_PP_LIST_FOR_EACH_PRODUCT_N_ ## i, FARM_PP_LIST_FOR_EACH_PRODUCT_I)
#
# if ~FARM_PP_CONFIG_FLAGS() & FARM_PP_CONFIG_MWCC()
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_H(data) FARM_PP_LIST_FOR_EACH_PRODUCT_H_I data
# else
#    define FARM_PP_LIST_FOR_EACH_PRODUCT_H(data) FARM_PP_LIST_FOR_EACH_PRODUCT_H_I(FARM_PP_TUPLE_ELEM(5, 0, data), FARM_PP_TUPLE_ELEM(5, 1, data), FARM_PP_TUPLE_ELEM(5, 2, data), FARM_PP_TUPLE_ELEM(5, 3, data), FARM_PP_TUPLE_ELEM(5, 4, data))
# endif
#
# define FARM_PP_LIST_FOR_EACH_PRODUCT_H_I(a, b, res, macro, size) (FARM_PP_LIST_FIRST(b), FARM_PP_LIST_REST(b), (FARM_PP_LIST_FIRST(a), res), macro, size)
#
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_0(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 0)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_1(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 1)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_2(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 2)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_3(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 3)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_4(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 4)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_5(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 5)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_6(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 6)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_7(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 7)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_8(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 8)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_9(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 9)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_10(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 10)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_11(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 11)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_12(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 12)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_13(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 13)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_14(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 14)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_15(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 15)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_16(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 16)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_17(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 17)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_18(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 18)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_19(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 19)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_20(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 20)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_21(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 21)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_22(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 22)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_23(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 23)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_24(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 24)(r, data)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_M_25(r, data) FARM_PP_LIST_FOR_EACH_PRODUCT_C(data, 25)(r, data)
#
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_0(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_1)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_1(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_2)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_2(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_3)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_3(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_4)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_4(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_5)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_5(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_6)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_6(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_7)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_7(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_8)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_8(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_9)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_9(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_10)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_10(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_11)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_11(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_12)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_12(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_13)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_13(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_14)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_14(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_15)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_15(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_16)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_16(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_17)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_17(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_18)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_18(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_19)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_19(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_20)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_20(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_21)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_21(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_22)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_22(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_23)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_23(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_24)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_24(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_25)
# define FARM_PP_LIST_FOR_EACH_PRODUCT_N_25(r, data) FARM_PP_FOR_ ## r(FARM_PP_LIST_FOR_EACH_PRODUCT_H(data), FARM_PP_LIST_FOR_EACH_PRODUCT_P, FARM_PP_LIST_FOR_EACH_PRODUCT_O, FARM_PP_LIST_FOR_EACH_PRODUCT_M_26)
#
# endif