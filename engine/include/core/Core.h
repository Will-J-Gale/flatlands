
template<typename CastPointer, typename Pointer>
inline CastPointer* Cast(Pointer* pointer)
{
    CastPointer* cast = dynamic_cast<CastPointer*>(pointer);
    assert(cast != nullptr);
    return cast;
};