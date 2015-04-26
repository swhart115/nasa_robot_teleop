/**
 * Put copyright notice here
 */
#include "RosparamDialog.hpp"
#include <QApplication>
#include <QDate>
#include <QDateTime>
#include <QDesktopWidget>
#include <QLayout>
#include <QPushButton>
#include <QRect>
#include <QTime>
#include <ostream>

namespace rviz_interactive_controls_panel {
	
	/**********************************************************************
	 * RosparamDialog
	 **********************************************************************/
	RosparamDialog::RosparamDialog(const std::string &ns, QWidget *parent)
			: QDialog(parent)
	{
		// set a maximum size based on default screen geometry
		const QRect screenRect = QApplication::desktop()->screenGeometry();
		setMaximumSize(screenRect.width(), screenRect.height());
		
		createRosparamTreeWidget(ns);
		m_ok = new QPushButton("OK", this);
		m_no = new QPushButton("Cancel", this);
		setupUi();
		connect(m_ok, SIGNAL(clicked()), this, SLOT(okClicked()));
		connect(m_no, SIGNAL(clicked()), this, SLOT(noClicked()));
	}
	
	RosparamDialog::~RosparamDialog() {
	}
	
	void RosparamDialog::okClicked() {
		processClick(QDialog::Accepted);
	}
	
	void RosparamDialog::noClicked() {
		processClick(QDialog::Rejected);
	}
	
	void RosparamDialog::onTreeWidthCalculated(int width) {
		ROS_INFO("RosparamDialog: tree width [%d]", width);
		if (width < maximumWidth()) {
			ROS_INFO("RosparamDialog: resizing to (%d,%d)", width, height());
			resize(QSize(width, height()));
		}
	}
	
	void RosparamDialog::createRosparamTreeWidget(const std::string &ns) {
		ROS_INFO("RosparamDialog: parameters from [%s]", ns.c_str());
		m_treeWidget = new RosparamTreeWidget(this);
		connect(m_treeWidget, SIGNAL(treeWidthCalculated(int)),
		        this, SLOT(onTreeWidthCalculated(int)));
		ros::NodeHandle nh;
		if (nh.getParam(ns, m_root)) {
			if (m_root.size() > 0) {
				m_treeWidget->fillTree(&m_root, ns);
			}
		} else {
			ROS_ERROR("RosparamDialog: [%s] not found!", ns.c_str());
		}
	}
	
	void RosparamDialog::processClick(QDialog::DialogCode code) {
		if (code == QDialog::Accepted) {
			ros::NodeHandle nh;
			m_treeWidget->commitUpdates(nh);
		}
		done(code);
	}
	
	void RosparamDialog::setupUi() {
		QHBoxLayout *but_lo = new QHBoxLayout();
		but_lo->addWidget(m_no);
		but_lo->addWidget(m_ok);
		
		QVBoxLayout *tot_lo = new QVBoxLayout();
		m_treeWidget->setMinimumSize(300, 200);
		tot_lo->addWidget(m_treeWidget);
		tot_lo->addLayout(but_lo);
		setLayout(tot_lo);
	}
	
	/**********************************************************************
	 * RosparamTreeWidget
	 **********************************************************************/
	int RosparamTreeWidget::rosparamItemType = 1001;
	
	RosparamTreeWidget::RosparamTreeWidget(RosparamDialog* parent)
			: QTreeWidget(parent)
	{
		setColumnCount(2);
		QStringList headers;
		headers << "Namespace Path" << "Value";
		setHeaderLabels(headers);
		setEditTriggers(QAbstractItemView::NoEditTriggers);
		connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)),
		        this, SLOT(onItemDoubleClicked(QTreeWidgetItem*, int)));
		connect(this, SIGNAL(itemChanged(QTreeWidgetItem*, int)),
		        this, SLOT(onItemChanged(QTreeWidgetItem*, int)));
	}
	
	RosparamTreeWidget::~RosparamTreeWidget() {
	}
	
	void RosparamTreeWidget::fillTree(XmlRpc::XmlRpcValue* rootVal,
	                                  const std::string &rootPath) {
		m_rootItem = new RosparamTreeItem(rootVal, NULL, rootPath);
		addTopLevelItem(m_rootItem);
		m_rootItem->setData(0, Qt::DisplayRole, QVariant(QString::fromStdString(rootPath)));
		expandAll();
		int wid0 = columnWidth(0), wid1 = columnWidth(1);
		ROS_INFO("RosparamTreeWidget: width 1=%d, width 2=%d", wid0, wid1);
		Q_EMIT treeWidthCalculated(wid0 + wid1);
		resizeColumnToContents(0);
		resizeColumnToContents(1);
	}
	
	void RosparamTreeWidget::commitUpdates(ros::NodeHandle &nh) {
		if (!m_updates.empty()) {
			std::unordered_set<RosparamTreeItem*>::iterator it;
			for (it=m_updates.begin(); it!=m_updates.end(); ++it) {
				(*it)->setParam(nh);
			}
		}
	}
	
	void RosparamTreeWidget::onItemDoubleClicked(QTreeWidgetItem* item, int col) {
		std::cout << "RosparamTreeWidget: doubleclick:";
		std::cout << " [" << item->text(0).toStdString() << "]";
		std::cout << ", col=" << col << std::endl;
		if (isEditable(col)) {
			editItem(item, col);
		}
	}
	
	void RosparamTreeWidget::onItemChanged(QTreeWidgetItem* item, int col) {
		std::cout << "RosparamTreeWidget: itemChanged:";
		std::cout << " [" << item->text(0).toStdString() << "]";
		std::cout << ", col=" << col << std::endl;
		if (isEditable(col)) {
			RosparamTreeItem* rpitem = static_cast<RosparamTreeItem*>(item);
			if (item->data(col, Qt::EditRole) != rpitem->data()) {
				std::cout << "  Values different! store for exit..." << std::endl;
				if (m_updates.count(rpitem) < 1) {
					m_updates.insert(rpitem);
					std::cout << "    Inserted " << rpitem;
					std::cout << " [" << item->text(0).toStdString() << "]";
					std::cout << std::endl;
				}
			} else {
				std::cout << "  Same values; ignore..." << std::endl;
				if (m_updates.count(rpitem) > 0) {
					m_updates.erase(rpitem);
					std::cout << "    removed " << rpitem;
					std::cout << " [" << item->text(0).toStdString() << "]";
					std::cout << std::endl;
				}
			}
		}
	}
	
	bool RosparamTreeWidget::isEditable(int col) {
		return (col == 1);
	}
	
	/**********************************************************************
	 * RosparamTreeItem
	 **********************************************************************/
	RosparamTreeItem::RosparamTreeItem(XmlRpc::XmlRpcValue* val,
	                                   RosparamTreeItem* parent,
	                                   const std::string &ns_path)
			: QTreeWidgetItem(parent, RosparamTreeWidget::rosparamItemType)
			, m_val(val)
			, m_nsPath(ns_path)
	{
		createChildren();
	}
	
	QVariant RosparamTreeItem::data() {
		return valToQVariant(*m_val);
	}
	
	bool RosparamTreeItem::setParam(const ros::NodeHandle &nh) {
		bool retval = false;
		QVariant qt_val = QTreeWidgetItem::data(1, Qt::EditRole);
		switch (m_val->getType()) {
		case XmlRpc::XmlRpcValue::TypeStruct:
		case XmlRpc::XmlRpcValue::TypeArray:
		case XmlRpc::XmlRpcValue::TypeInvalid:
			ROS_ERROR("RosparamTreeItem: attempt to set invalid value!");
			break;
		case XmlRpc::XmlRpcValue::TypeBoolean:
			if (qt_val.canConvert(QVariant::Bool)) {
				(bool&)(*m_val) = qt_val.toBool();
				retval = true;
			}
			break;
		case XmlRpc::XmlRpcValue::TypeInt:
			if (qt_val.canConvert(QVariant::Int)) {
				(int&)(*m_val) = qt_val.toInt();
				retval = true;
			}
			break;
		case XmlRpc::XmlRpcValue::TypeDouble:
			if (qt_val.canConvert(QVariant::Double)) {
				(double&)(*m_val) = qt_val.toDouble();
				retval = true;
			}
			break;
		case XmlRpc::XmlRpcValue::TypeString:
			if (qt_val.canConvert(QVariant::String)) {
				(std::string&)(*m_val) = qt_val.toString().toStdString();
				retval = true;
			}
			break;
		case XmlRpc::XmlRpcValue::TypeDateTime:
			if (qt_val.canConvert(QVariant::DateTime)) {
				QDateTime qdt = qt_val.toDateTime();
				if (qdt.isValid()) {
					struct tm time;
					time.tm_sec   = qdt.time().second();
					time.tm_min   = qdt.time().minute();
					time.tm_hour  = qdt.time().hour();
					time.tm_mday  = qdt.date().day();
					time.tm_mon   = qdt.date().month() - 1;
					time.tm_year  = qdt.date().year() - 1900;
					time.tm_wday  = qdt.date().dayOfWeek() - 1;
					time.tm_yday  = qdt.date().dayOfYear() - 1;
					time.tm_isdst = -1;
					(struct tm&)(*m_val) = time;
					retval = true;
				}
			}
			break;
		case XmlRpc::XmlRpcValue::TypeBase64:
			if (qt_val.canConvert(QVariant::ByteArray)) {
				QByteArray qba = qt_val.toByteArray();
				XmlRpc::XmlRpcValue::BinaryData bd;
				std::copy(qba.data(), qba.data() + qba.size(), bd.begin());
				(XmlRpc::XmlRpcValue::BinaryData&)(*m_val) = bd;
				retval = true;
			}
			break;
		default:
			break;
		}
		if (retval) {
			nh.setParam(m_nsPath, *m_val);
		}
		return retval;
	}
	
	void RosparamTreeItem::createChildren() {
		if (m_val->getType() == XmlRpc::XmlRpcValue::TypeStruct) {
			XmlRpc::XmlRpcValue::iterator it;
			for (it=m_val->begin(); it!=m_val->end(); ++it) {
				addChild(it->first, &(it->second));
			}
		} else if (m_val->getType() == XmlRpc::XmlRpcValue::TypeArray) {
			for (int i=0; i<m_val->size(); ++i) {
				addChild("", &((*m_val)[i]));
			}
		}
	}
	
	void RosparamTreeItem::addChild(const std::string &name,
	                                XmlRpc::XmlRpcValue* val) {
		std::string path = ros::names::append(this->m_nsPath, name);
		if (name.empty()) {
			path = name;   // no path
		}
		RosparamTreeItem* item = new RosparamTreeItem(val, this, path);
		item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable);
		item->setData(0, Qt::DisplayRole, QVariant(QString::fromStdString(name)));
		item->setData(1, Qt::EditRole, valToQVariant(*val));
		//item->setSizeHint(0, QSize(
		QTreeWidgetItem::addChild(item);
		item->setExpanded(true);
	}
	
	QVariant RosparamTreeItem::valToQVariant(XmlRpc::XmlRpcValue &val) const {
		QVariant retval;
		switch(val.getType()) {
		case XmlRpc::XmlRpcValue::TypeBoolean:
			retval = QVariant((bool)val);
			break;
		case XmlRpc::XmlRpcValue::TypeInt:
			retval = QVariant((int)val);
			break;
		case XmlRpc::XmlRpcValue::TypeDouble:
			retval = QVariant((double)val);
			break;
		case XmlRpc::XmlRpcValue::TypeString:
			retval = QVariant(((std::string)val).c_str());
			break;
		case XmlRpc::XmlRpcValue::TypeDateTime:
			{
				struct tm time = (struct tm)val;
				int ms = 0;
				if (time.tm_sec > 59) {
					// urgh...leap seconds; just discard (!)
					time.tm_sec = 59;
					ms = 999;
				}
				QDateTime qdt(
				      QDate(time.tm_year + 1900, time.tm_mon + 1, time.tm_mday),
				      QTime(time.tm_hour, time.tm_min, time.tm_sec, ms));
				retval = QVariant(qdt);
				break;
			}
		case XmlRpc::XmlRpcValue::TypeBase64:
			{
				std::vector<char> &bd = (XmlRpc::XmlRpcValue::BinaryData&)val;
				QByteArray ba(bd.data(), bd.size());
				retval = QVariant(ba);
				break;
			}
		default:
			retval = QVariant();
		}
		return retval;
	}
	
}
