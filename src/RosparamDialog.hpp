/**
 * Put copyright notice here
 */
#ifndef ROSPARAM_DIALOG_HPP
#define ROSPARAM_DIALOG_HPP

#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <QDialog>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <unordered_set>

namespace rviz_interactive_controls_panel {
	
	class RosparamTreeWidget;
	class RosparamTreeItem;
	
	class RosparamDialog : public QDialog {
		Q_OBJECT
		public:
			RosparamDialog(const std::string &ns, QWidget *parent = 0);
			~RosparamDialog();
		private Q_SLOTS:
			/** Process a click of the \a OK button (accepts any changes). */
			void okClicked();
			/** Process a click of the \a Cancel button (discards changes). */
			void noClicked();
		private:
			/** Create the \c RosparamTreeWidget, using \c ns as the root. */
			void createRosparamTreeWidget(const std::string &ns);
			/** Executed prior to closing; updates any changed parameter
			 * values. */
			void processClick(QDialog::DialogCode code);
			void setupUi();
			
			XmlRpc::XmlRpcValue m_root;
			RosparamTreeWidget *m_treeWidget;
			QPushButton *m_ok, *m_no;
	};
	
	class RosparamTreeWidget : public QTreeWidget {
		Q_OBJECT
		public:
			RosparamTreeWidget(RosparamDialog* parent = 0);
			~RosparamTreeWidget();
			void fillTree(XmlRpc::XmlRpcValue* rootVal,
			              const std::string &rootPath);
			void commitUpdates(ros::NodeHandle &nh);
			static int rosparamItemType;
		private Q_SLOTS:
			/** Handle item editing (on double click). */
			void onItemDoubleClicked(QTreeWidgetItem* item, int col);
			/** Record that items changed for update on exit. */
			void onItemChanged(QTreeWidgetItem* item, int col);
		private:
			bool isEditable(int col);
			RosparamTreeItem* m_rootItem;
			std::unordered_set<RosparamTreeItem*> m_updates;
	};
	
	class RosparamTreeItem : public QTreeWidgetItem {
		public:
			/** Constructor. */
			RosparamTreeItem(XmlRpc::XmlRpcValue* val,
			                 RosparamTreeItem* parent,
			                 const std::string &path);
			QVariant data();
			bool setParam(const ros::NodeHandle &nh);
		private:
			void createChildren();
			void addChild(const std::string &name,
			              XmlRpc::XmlRpcValue* xmlVal);
			QVariant valToQVariant(XmlRpc::XmlRpcValue &val) const;
			
			XmlRpc::XmlRpcValue* m_val;
			std::string m_nsPath;
	};
	
}
#endif
