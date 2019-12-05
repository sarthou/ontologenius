#ifndef ONTOLOGENIUS_LINECOUNTER_H
#define ONTOLOGENIUS_LINECOUNTER_H

namespace ontologenius {

class LinesCounter
{
public:
  LinesCounter() { start_ = stop_ = 0; nb_lines_ = 1; }
  size_t getStart() {return start_;}
  size_t getStop() {return stop_;}
  size_t getNbLines() {return (nb_lines_ = (stop_ - start_ + 1));}
  void setStart(size_t start) {start_ = start;}
  void setStop(size_t stop) {stop_ = stop;}

  size_t current_line_;

private:
  size_t start_;
  size_t stop_;
  size_t nb_lines_;
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_LINECOUNTER_H
