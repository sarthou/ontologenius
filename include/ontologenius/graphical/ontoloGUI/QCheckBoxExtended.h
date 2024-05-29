#ifndef QCHECKBOXEXTENDED_H
#define QCHECKBOXEXTENDED_H

#include <QCheckBox>

class QCheckBoxExtended : public QCheckBox
{
  Q_OBJECT
public:
  explicit QCheckBoxExtended(QWidget* parent = 0);
  explicit QCheckBoxExtended(const QString& text, QWidget* parent = 0);
  virtual ~QCheckBoxExtended() = default;

protected:
  void hoverEnter(QHoverEvent* event);
  void hoverLeave(QHoverEvent* event);
  void hoverMove(QHoverEvent* event);
  bool event(QEvent* event);

Q_SIGNALS:
  void hoverEnter();
  void hoverLeave();

public slots:
};

#endif // QCHECKBOXEXTENDED_H
