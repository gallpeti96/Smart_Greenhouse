import sqlite3
from datetime import datetime

def create_connection(db_file):
    conn = None
    conn = sqlite3.connect(db_file, check_same_thread=False)
    print("sqlite version: ", sqlite3.version)

    return conn


def create_table(conn, create_table_sql):
    c = conn.cursor()
    c.execute(create_table_sql)


def create_record(conn, table, data):
    time = (datetime.now(),)
    data = time + data
    if table == 'Measurements':
        sql = ''' INSERT INTO Measurements(timestamp, sensorID, SensorValue) VALUES(?,?,?) '''
    elif table == 'Communications':
        sql = ''' INSERT INTO Communications(timestamp, LastSent, LastReceived) VALUES(datetime.now(),?,?) '''
    else:
        print("wrong table name!")

    cur = conn.cursor()
    cur.execute(sql, data)
    conn.commit()
    return cur.lastrowid

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
                                    timestamp DATETIME,
                                    LastSent integer,
                                    LastReceived integer,
                                    FOREIGN KEY (LastSent) REFERENCES Measurements (id)
                                    FOREIGN KEY (LastReceived) REFERENCES Measurements (id)
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
