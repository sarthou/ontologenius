/*
###############################################################################
#                                                                             #
# The MIT License                                                             #
#                                                                             #
# Copyright (C) 2017 by Juergen Skrotzky (JorgenVikingGod@gmail.com)          #
#               >> https://github.com/Jorgen-VikingGod                        #
#                                                                             #
# Sources: https://github.com/Jorgen-VikingGod/Qt-Frameless-Window-DarkStyle  #
#                                                                             #
###############################################################################
*/

#include "include/ontologenius/graphical/ontoloGUI/DarkStyle.h"

#include "qapplication.h"
#include "qfont.h"
#include "qiodevice.h"
#include "qnamespace.h"
#include "qpalette.h"
#include "qproxystyle.h"
#include "qstring.h"
#include "qstyle.h"
#include "qstylefactory.h"

#ifndef QStringLiteral
#define QStringLiteral(str) QString(str)
#endif

DarkStyle::DarkStyle() : DarkStyle(styleBase())
{}

DarkStyle::DarkStyle(QStyle* style) : QProxyStyle(style)
{}

QStyle* DarkStyle::styleBase(QStyle* style) const
{
  static QStyle* base_ = (style == nullptr) ? QStyleFactory::create(QStringLiteral("Fusion")) : style; // NOLINT
  return base_;
}

QStyle* DarkStyle::baseStyle() const
{
  return styleBase();
}

void DarkStyle::polish(QPalette& palette)
{
  // modify palette to dark
  palette.setColor(QPalette::Window, QColor(53, 53, 53));
  palette.setColor(QPalette::WindowText, Qt::white);
  palette.setColor(QPalette::Disabled, QPalette::WindowText, QColor(127, 127, 127));
  palette.setColor(QPalette::Base, QColor(42, 42, 42));
  palette.setColor(QPalette::AlternateBase, QColor(66, 66, 66));
  palette.setColor(QPalette::ToolTipBase, Qt::white);
  palette.setColor(QPalette::ToolTipText, QColor(53, 53, 53));
  palette.setColor(QPalette::Text, Qt::white);
  palette.setColor(QPalette::Disabled, QPalette::Text, QColor(127, 127, 127));
  palette.setColor(QPalette::Dark, QColor(35, 35, 35));
  palette.setColor(QPalette::Shadow, QColor(20, 20, 20));
  palette.setColor(QPalette::Button, QColor(53, 53, 53));
  palette.setColor(QPalette::ButtonText, Qt::white);
  palette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(127, 127, 127));
  palette.setColor(QPalette::BrightText, Qt::red);
  palette.setColor(QPalette::Link, QColor(42, 130, 218));
  palette.setColor(QPalette::Highlight, QColor(42, 130, 218));
  palette.setColor(QPalette::Disabled, QPalette::Highlight, QColor(80, 80, 80));
  palette.setColor(QPalette::HighlightedText, Qt::white);
  palette.setColor(QPalette::Disabled, QPalette::HighlightedText, QColor(127, 127, 127));
}

void DarkStyle::polish(QApplication* app)
{
  if(app == nullptr)
    return;

  // increase font size for better reading,
  // setPointSize was reduced from +2 because when applied this way in Qt5, the font is larger than intended for some reason
  QFont default_font = QApplication::font();
  default_font.setPointSize(default_font.pointSize() + 1);
  QApplication::setFont(default_font);

  // loadstylesheet
  QFile qf_darkstyle(QStringLiteral(":/darkstyle/darkstyle.qss")); // NOLINT
  if(qf_darkstyle.open(QIODevice::ReadOnly | QIODevice::Text))
  {
    // set stylesheet
    const QString qs_stylesheet = QString::fromLatin1(qf_darkstyle.readAll());
    app->setStyleSheet(qs_stylesheet);
    qf_darkstyle.close();
  }
}
