import sqlite3
from datetime import datetime
import threading

def create_connection(db_file):
    conn = None
    conn = sqlite3.connect(db_file, check_same_thread=False)    # each thread needs to add new rows
    print("sqlite version: ", sqlite3.version)

    return conn


def create_table(conn, create_table_sql):
    c = conn.cursor()
    c.execute(create_table_sql)


def create_record(conn, table, data):
    time = (datetime.now(),)
    if table == 'Measurements':
        data = time + data
        sql = '''INSERT INTO Measurements(timestamp, sensorID, SensorValue) VALUES(?,?,?) '''
        cur = conn.cursor()
        cur.execute(sql, data)
    elif table == 'Communications':
        to_log = []
        not_sent = data[0]
        timestamp = data[1]
        sql = '''INSERT INTO Communications(meas_timestamp, sent_timestamp) VALUES(?,?)'''
        for i in range(0, len(not_sent)):
            to_log.append((not_sent[i][1], timestamp))
        cur = conn.cursor()
        cur.executemany(sql, to_log)

    else:
        print("wrong table name!")

    conn.commit()
    return cur.lastrowid


def select_last_entry(conn):
    sql = '''SELECT max(id) from Communications '''
    cur = conn.cursor()
    cur.execute(sql)
    return cur.fetchall()


def get_not_sends(conn, last):
    sql = '''SELECT * FROM Measurements WHERE id > ''' + last
    cur = conn.cursor()
    cur.execute(sql)
    return cur.fetchall()

def create_db(name):
    database = r"/home/pi/db/"+name

    sql_create_measurements_table = """ CREATE TABLE IF NOT EXISTS Measurements (
                                        id integer PRIMARY KEY,
                                        timestamp DATETIME,
                                        sensorID text,
                                        SensorValue real
                                    ); """

    sql_create_communications_table = """CREATE TABLE IF NOT EXISTS Communications (
                                    id integer PRIMARY KEY,
                                    meas_timestamp DATETIME,
                                    sent_timestamp DATETIME,
                                    FOREIGN KEY (meas_timestamp) REFERENCES Measurements (timestamp)
                                );"""

    conn = create_connection(database)

    # create tables
    if conn is not None:
        # create projects table
        create_table(conn, sql_create_measurements_table)

        # create tasks table
        create_table(conn, sql_create_communications_table)
    else:
        print("Error! cannot create the database connection.")
    return conn
