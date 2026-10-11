#pragma once
// ============================================================
//  TemplateTraits.hpp
//  Utilidades de meta-programación y estructuras de despacho
//  de mensajes (MemberAction, ForEachAction, etc.)
//
//  Requiere C++17 (-std=c++17) — usa if constexpr, fold
//  expressions y template<auto> (NTTP). En Arduino IDE agregar
//  al board's platform.txt:
//    compiler.cpp.extra_flags=-std=c++17
// ============================================================

#include "Messaging.hpp"   // Writer, Reader, MsgHandler

template<void (**handlr)(Writer&, Reader&), int max>
void handlerCaller(Writer& w, Reader& r);

// ============================================================
//  Traits internos
// ============================================================
namespace traits {

    template<typename T, typename U>
    struct is_same { static constexpr bool value = false; };
    template<typename T>
    struct is_same<T, T> { static constexpr bool value = true; };

    template<typename... Ts>
    struct type_list {};

    // ---- is_member_function_pointer ----------------------------
    template<typename>
    struct is_member_function_pointer {
        static constexpr bool value = false;
        using return_type = void;
        using class_type  = void;
        using arg_types   = type_list<>;
    };
    template<typename R, typename C, typename... Args>
    struct is_member_function_pointer<R (C::*)(Args...)> {
        static constexpr bool value = true;
        using return_type = R;
        using class_type  = C;
        using arg_types   = type_list<Args...>;
    };
    template<typename R, typename C, typename... Args>
    struct is_member_function_pointer<R (C::*)(Args...) const> {
        static constexpr bool value = true;
        using return_type = R;
        using class_type  = C;
        using arg_types   = type_list<Args...>;
    };

    // ---- is_void -----------------------------------------------
    template<typename>    struct is_void        { static constexpr bool value = false; };
    template<>            struct is_void<void>   { static constexpr bool value = true;  };

    // ---- is_ptr ------------------------------------------------
    template<typename>
    struct is_ptr {
        static constexpr bool    value = false;
        static constexpr uint8_t star  = 0;
        using type = void;
    };
    template<typename T> struct is_ptr<T*> {
        static constexpr bool    value = true;
        static constexpr uint8_t star  = 1;
        using type = T;
    };
    template<typename T> struct is_ptr<T**> {
        static constexpr bool    value = true;
        static constexpr uint8_t star  = 2;
        using type = T;
    };

    // ---- is_reference ------------------------------------------
    template<typename>   struct is_reference      { static constexpr bool value = false; };
    template<typename T> struct is_reference<T&>  { static constexpr bool value = true;  };

    // ---- declval -----------------------------------------------
    template<typename T> T&& declval() noexcept;

    // ---- has_static_call ---------------------------------------
    template<typename F, typename... Args>
    struct has_static_call {
    private:
        template<typename U>
        static auto test(int) -> decltype(U::call(declval<Args>()...), char{});
        template<typename>
        static long test(...);
    public:
        static constexpr bool value = sizeof(test<F>(0)) == sizeof(char);
    };

    // ---- member ------------------------------------------------
    template<typename... Args, typename C, typename R>
    constexpr auto member(R (C::*p)(Args...))       { return p; }
    template<typename... Args, typename C, typename R>
    constexpr auto member(R (C::*p)(Args...) const) { return p; }

    // ---- conditional -------------------------------------------
    template<bool Cond, typename T, typename F>
    struct conditional              { using type = T; };
    template<typename T, typename F>
    struct conditional<false, T, F> { using type = F; };

    // ---- is_base_of --------------------------------------------
    // true si Base es base-o-igual de Derived.
    // Truco: Derived* convierte implícitamente a Base* si hay herencia.
    template<typename Base, typename Derived>
    struct is_base_of {
    private:
        static char test(const Base*);
        static long test(...);
    public:
        static constexpr bool value =
            sizeof(test(static_cast<Derived*>(nullptr))) == sizeof(char);
    };

    template<auto first, auto... Members>
    struct get_first_member_type{
        using type = typename is_member_function_pointer<decltype(first)>::class_type;
    };

} // namespace traits

using namespace traits;

namespace GenerateFunctionsImpl{
    // ============================================================
    //  MemberActionUnit
    //  Adapta un único puntero a miembro.
    //  call<T> recibe T* (puede ser más derivado que type);
    //  el upcast a type* lo hace C++ implícitamente.
    // ============================================================
    template<auto Member, typename ArgList>
    struct MemberActionUnitImpl {};

    template<auto Member, typename... Args>
    struct MemberActionUnitImpl<Member, type_list<Args...>> {
        static_assert(is_member_function_pointer<decltype(Member)>::value,
            "Member no es una funcion miembro");

        using Ret  = typename is_member_function_pointer<decltype(Member)>::return_type;
        using type = typename is_member_function_pointer<decltype(Member)>::class_type;

        static constexpr bool unwritable =
            is_void<Ret>::value || is_ptr<Ret>::value || is_reference<Ret>::value;

        template<typename T>
        static void call(Writer& w, Reader& r, T* obj) {
            if (!obj) {
                if constexpr (sizeof...(Args) > 0) r.skip((sizeof(Args) + ...));
                return;
            }
            if constexpr (sizeof...(Args) > 0) {
                if (!r.hasSpace((sizeof(Args) + ...))) return;
                if constexpr (unwritable)
                    (obj->*Member)(r.readSafe<Args>()...);
                else
                    if (!w.write((obj->*Member)(r.readSafe<Args>()...))) w.info = false;
            } else {
                if constexpr (unwritable)
                    (obj->*Member)();
                else
                    if (!w.write((obj->*Member)())) w.info = false;
            }
        }
    };

    template<auto Member>
    struct MemberActionUnit : MemberActionUnitImpl<Member,
        typename is_member_function_pointer<decltype(Member)>::arg_types> {};


    // ============================================================
    //  MemberActionImpl — implementación con T ya resuelto
    // ============================================================
    template<typename T, typename... Fs>
    struct MemberActionImpl {
        static_assert(sizeof...(Fs) > 0,
            "Se debe dar al menos un puntero a funcion miembro");
        static_assert((is_base_of<typename Fs::type, T>::value && ...),
            "Todos los class_type deben ser base-o-igual del tipo T, (utiliza ExplicitMemberAction si no sabes porque se suelta este error)");

        using type = T;

        static void call(Writer& w, Reader& r, T* obj) {
            ((Fs::template call<T>(w, r, obj), ...));
        }
    };
}
using namespace GenerateFunctionsImpl;

// ============================================================
//  MemberAction
//
//  Caso normal — T deducido del pack (incluye herencia múltiple
//  si el tipo hoja aparece entre los miembros):
//    MemberAction<&Base::foo, &Derived::bar>
//    MemberAction<&UpdBase::update, &SerBase::write, &Motor::tick>
//
//  Herencia múltiple sin hoja en el pack — T explícito mínimo:
//    MemberAction<Motor, &UpdBase::update, &SerBase::write>
// ============================================================

// T explícito (primer parámetro es un tipo)
template<typename T, auto... Members>
struct ExplicitMemberAction : MemberActionImpl<T, MemberActionUnit<Members>...> {};

// T deducido (solo punteros a miembro)
template<auto... Members>
struct MemberAction : MemberActionImpl<
    typename get_first_member_type<Members...>::type,
    MemberActionUnit<Members>...> {};


// ============================================================
//  InstanceAction
//  Llama a Fn::call sobre un puntero a instancia fijo (Elem).
//  SkipRead=true restaura el cursor del Reader tras la llamada.
// ============================================================
template<auto Elem, typename Fn, bool SkipRead = true>
struct InstanceAction {
    static_assert(is_ptr<decltype(Elem)>::star == 1,
        "Elem debe ser un puntero simple (T*)");
    using T = typename is_ptr<decltype(Elem)>::type;
    static_assert(has_static_call<Fn, Writer&, Reader&, T*>::value,
        "Fn no tiene la funcion static void call(Writer&, Reader&, T*)");

    static void call(Writer& w, Reader& r) {
        if constexpr (SkipRead) {
            size_t place { r.getCursor() };
            Fn::call(w, r, Elem);
            r.setCursor(place);
        } else {
            Fn::call(w, r, Elem);
        }
    }
};


// ============================================================
//  ForEachAction
//  Itera sobre Arr[Count] llamando Fn::call para cada elemento.
//  Masked=true    lee un bitmask; salta elementos con bit 0.
//  SameInput=true restaura el cursor entre elementos.
//  SkipRead=true  avanza el cursor al final del payload al terminar.
// ============================================================
template<auto Arr, size_t Count, typename Fn,
         bool Masked    = false,
         bool SameInput = true,
         bool SkipRead  = true>
struct ForEachAction {
    static_assert(is_ptr<decltype(Arr)>::star == 2,
        "Arr debe ser un puntero a puntero (T**)");
    using T = typename is_ptr<decltype(Arr)>::type;
    static_assert(has_static_call<Fn, Writer&, Reader&, T*>::value,
        "Fn no tiene la funcion static void call(Writer&, Reader&, T*)");

    static void call(Writer& w, Reader& r) {
        size_t place { r.getCursor() };
        size_t dif   { 0             };

        if constexpr (Masked) {
            uint8_t mask = r.readSafe(uint8_t{});
            place = r.getCursor();
            if (mask) {
                for (size_t i = 0; i < Count; (i++, mask >>= 1)) {
                    if (!(mask & 1)) continue;
                    Fn::call(w, r, Arr[i]);
                    if constexpr (SameInput)
                        dif = r.setCursor(place) - place;
                }
            } else {
                if constexpr (SkipRead) Fn::call(w, r, nullptr);
                return;
            }
        } else {
            for (size_t i = 0; i < Count; i++) {
                Fn::call(w, r, Arr[i]);
                if constexpr (SameInput)
                    dif = r.setCursor(place) - place;
            }
        }
        if constexpr (SkipRead) r.skip(dif);
    }
};


// ============================================================
//  ActionSequence
// ============================================================
template<typename... Fs>
struct ActionSequence {
    static_assert((has_static_call<Fs, Writer&, Reader&>::value && ...),
        "Una de las clases no tiene la funcion static void call(Writer&, Reader&)");

    static void call(Writer& w, Reader& r) {
        ((Fs::call(w, r), ...));
    }
};


// ============================================================
//  HandlerEntry
// ============================================================
template<typename Fn, bool Send = true>
struct HandlerEntry {
    static void call(Writer& w, Reader& r) {
        Fn::call(w, r);
        if constexpr (!Send) w.info = false;
    }
};


// ============================================================
//  handlerCaller
// ============================================================
template<void (**handlr)(Writer&, Reader&), int max>
void handlerCaller(Writer& w, Reader& r) {
    if (r.isDone()) return;
    uint8_t inst = r.readSafe(uint8_t{});
    if (inst >= (uint8_t)max) return;
    if (handlr[inst]) {
        w.write(inst);
        handlr[inst](w, r);
    } else {
        w.info = false;
    }
}


// ============================================================
//  registerHandler
// ============================================================
template<typename Fn, bool Send = true>
void registerHandler(void (**handler)(Writer&, Reader&), uint8_t id) {
    static_assert(has_static_call<Fn, Writer&, Reader&>::value,
        "Fn no tiene la funcion static void call(Writer&, Reader&)");
    handler[id] = HandlerEntry<Fn, Send>::call;
}