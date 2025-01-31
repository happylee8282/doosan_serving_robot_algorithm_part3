import sqlite3
from datetime import datetime
from matplotlib import pyplot as plt
import numpy as np
import sys
from PyQt5.QtWidgets import QApplication, QWidget
import threading

from PyQt5.QtWidgets import QApplication, QMainWindow, QCalendarWidget, QLabel, QVBoxLayout, QWidget, QMessageBox, QPushButton
from PyQt5.QtCore import QDate, Qt
from PyQt5.QtGui import QTextCharFormat

from PyQt5.QtGui import QPixmap



# 달력 보여주고, 날짜를 클릭하면 그에 대한 통계가 나옴.
class DBGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.initUI()

        conn = sqlite3.connect('/home/happy/Desktop/menu.db')
        cursor = conn.cursor()

        order = 'Select * from menu'
        cursor.execute(order)
        # 모든 내용 긁어오기
        result = cursor.fetchall()

        conn.commit()
        conn.close()

        self.food = []
        self.money = []

        for i in result:
            self.food.append(i[0])
            self.money.append(i[2])






    def initUI(self):
        self.setWindowTitle("QCalendarWidget Example")
        self.setGeometry(100, 100, 400, 300)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)

        self.calendar = QCalendarWidget(self)
        self.calendar.setGridVisible(True)
        self.calendar.setNavigationBarVisible(True)
        self.calendar.setFirstDayOfWeek(Qt.Monday)
        self.calendar.setSelectionMode(QCalendarWidget.SingleSelection)
        self.calendar.setMinimumDate(QDate(2000, 1, 1))
        self.calendar.setMaximumDate(QDate(2100, 12, 31))
        self.calendar.setHorizontalHeaderFormat(QCalendarWidget.SingleLetterDayNames)
        self.calendar.setVerticalHeaderFormat(QCalendarWidget.ISOWeekNumbers)
        
        self.calendar.clicked.connect(self.on_date_clicked)
        self.layout.addWidget(self.calendar)

        self.label = QLabel("Selected Date: None", self)
        self.layout.addWidget(self.label)

        # 특정 날짜 강조
        self.highlight_date(QDate(2023, 12, 25), Qt.red)

        self.show()

    # 클릭하면 통계 보여주기
    def on_date_clicked(self, date):
        self.label.setText(f"Selected Date: {date.toString()}")
        data = date.toString()
        self.sub_k_callback(data)

    # 클릭한 날짜 색깔 다르게
    def highlight_date(self, date, color):
        format = QTextCharFormat()
        format.setForeground(color)
        self.calendar.setDateTextFormat(date, format)

    #실제 통계 처리 되는 부분
    def sub_k_callback(self, data):
        # db에 연결
        conn = sqlite3.connect('/home/happy/Desktop/newfile/order.db')
        cursor = conn.cursor()

        # data.data는 2025_01_24 형태
        d = data.split(' ')

        # 1달치
        month = d[3]+'_01'
        

        month_day = np.arange(1,32,1)
        month_whole_per_day = np.zeros(31)

        for i in range(1,32,1):
            try:
                if len(str(i)) == 2:
                    month = month + '_' + str(i)
                else:
                    month = month + '_0' + str(i)

                month_order = 'Select * from orders'+month
                cursor.execute(month_order)
                month_result = cursor.fetchall()

                conn.commit()

                for j in month_result:
                    month_whole_per_day[i-1] += int(j[2]) * int(self.money[self.food.index(j[1])])

                month = month[:7]
            except:
                month = month[:7]
        
        month_whole = sum(month_whole_per_day)
        # 1주일치



        # 하루치

        a = d[3]+'_01'+'_'+d[2]

        # 선택된 날짜에 맞는 table선택
        order = 'Select * from orders'+a

        cursor.execute(order)
        # 모든 내용 긁어오기
        result = cursor.fetchall()

        conn.commit()
        conn.close()

        # 음식 리스트
        fo = []
        # 종합 금액
        mo = []
        # 총합 갯수
        co = []
        # 총액
        whole = 0

        # 리스트에 없으면 append, 있으면 위치에 맞게 갯수 증가
        for i in result:
            if i[1] in fo:
                co[fo.index(i[1])] += int(i[2])
                mo[fo.index(i[1])] += int(i[2]) * int(self.money[self.food.index(i[1])])

            else:
                fo.append(i[1])
                co.append(int(i[2]))
                mo.append(int(i[2]) * int(self.money[self.food.index(i[1])]))

        for i in range(len(fo)):
            whole += mo[i]


        me = ''
        for i in range(len(fo)):
            me = me + fo[i] + ' :    ' + str(co[i]) + '개  -   ' + str(mo[i]) + '원\n'
        me = me + '\n가장 잘 팔린 메뉴 : ' + fo[co.index(max(co))] + '\n\n\n'
        me = me + '\n일간 총 판매액 : ' + str(whole) + '원\n\n'
        me = me + '월간 총 판매액 : ' + str(month_whole) + '원'

        

        # 시간대별로 나누기 위해서(영업시간 10:00:00 - 21:59:59, 2시간 단위)
        food = []
        count_by_time = []
        whole_by_time = [0, 0, 0, 0, 0, 0]
        ti = np.arange(10, 22, 2)

        for i in result:
            hour = int(i[0].split(':')[0])
            ind = hour//2-5

            whole_by_time[ind] += int(i[2])

            if i[1] in food:
                count_by_time[food.index(i[1])][ind] += int(i[2]) * int(self.money[self.food.index(i[1])])
            else:
                food.append(i[1])
                count_by_time.append([0, 0, 0, 0, 0, 0])
                count_by_time[food.index(i[1])][ind] = int(i[2]) * int(self.money[self.food.index(i[1])])

        for i in count_by_time:
            for j in range(len(i)):
                whole_by_time[j] += i[j]


        # 그래프 그리기
        # 월간 일별 매출
        for i in range(len(month_day)):
            plt.plot(month_day, month_whole_per_day)
        plt.grid()
        plt.title(f'{month} sales per day')
        plt.savefig('month.png')

        plt.figure(figsize=(6,8))

        plt.cla()

        # 1일 시간별 총 매출
        plt.plot(ti, whole_by_time, label = 'whole')
        plt.title(f'{a} sales per hour whole')
        
        # 1일 시간별 메뉴 총 매출
        for i in range(len(food)):
            plt.plot(ti, count_by_time[i], label = food[i] )
        plt.grid()
        plt.legend()
        plt.title(f'{a} sales per hour')
        
        plt.savefig('1day.png')

        plt.cla()

        pixmap_m = QPixmap('month.png')
        lbl_img_m = QLabel()
        lbl_img_m.setPixmap(pixmap_m)
        lbl_img_m.show()


        pixmap = QPixmap('1day.png')
        lbl_img = QLabel()
        lbl_img.setPixmap(pixmap)
        lbl_img.show()

        # 팝업창 관련
        messagebox = QMessageBox()
        messagebox.setWindowTitle(f"{a} 판매액")
        messagebox.setText(f"{me}")

        ok_button = QPushButton("OK")
        messagebox.addButton(ok_button, QMessageBox.AcceptRole)
        messagebox.exec_()


def main(args=None):
    app = QApplication(sys.argv)  # PyQt 애플리케이션 생성

    gui = DBGUI(None)  # GUI 생성
    app.exec_()  # PyQt 이벤트 루프 실행

if __name__ == '__main__':
    main()  # 메인 함수 실행