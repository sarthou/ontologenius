#ifndef ONTOLOGENIUS_TRIPLET_H
#define ONTOLOGENIUS_TRIPLET_H

namespace ontologenius {

template<typename S, typename P, typename O>
struct Triplet_t
{
	Triplet_t(S* s, P* p, O* o) : subject(s), predicate(p), object(o) {}
	S* subject;
	P* predicate;
	O* object;

	bool operator==(const Triplet_t& other)
	{
		return ((subject == other.subject) &&
						(predicate == other.predicate) &&
						(object == other.object));
	}

	bool equals(S* s, P* p, O* o)
	{
		return ((subject == s) &&
						(predicate == p) &&
						(object == o));
	}
};

class TripletsInterface
{
public:
	virtual ~TripletsInterface() = default;
	virtual bool eraseGeneric(void* s, void* p, void* o) = 0;
};

template<typename S, typename P, typename O>
class Triplets : public TripletsInterface
{
public:
	void push(S* subject,
						P* predicate,
						O* object)
	{
		triplets.emplace_back(subject, predicate, object);
	}

	bool exist(S* subject,
						P* predicate,
						O* object)
	{
		return(std::find_if(triplets.begin(), triplets.end(),
												[subject, predicate, object](auto& triplet)
												{return triplet.equals(subject, predicate, object);})
												!= triplets.end());
	}
	bool eraseGeneric(void* s, void* p, void* o) override
	{
		return erase((S*)s, (P*)p, (O*)o);
	}

	std::vector<Triplet_t<S,P,O>> triplets;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_TRIPLET_H