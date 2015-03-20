#ifndef MESS_WIDGET_H
#define MESS_WIDGET_H

#include <QWidget>
#include "SurfaceMeshPlugins.h"

class mess_mode;

namespace Ui {
class mess_widget;
}

class mess_widget : public QWidget
{
    Q_OBJECT

public:
    explicit mess_widget(mess_mode * m = 0);
    ~mess_widget();

	QString loadScanSceneName();
	QString loadActionJobName();

public slots:


private:
    Ui::mess_widget *ui;
	mess_mode *m_mode;
};

#endif // MESS_WIDGET_H
