DROP DATABASE DAISSOTORAGY; 

CREATE DATABASE DAISSOTORAGY;

USE DAISSOTORAGY;

CREATE TABLE SECTION (
	ID INT NOT NULL AUTO_INCREMENT,
    NAME VARCHAR(20) NOT NULL,
    PRIMARY KEY (ID)
);

CREATE TABLE ITEM (
	ID INT NOT NULL AUTO_INCREMENT,
    NAME VARCHAR(20) NOT NULL,
    SECTION_ID INT NOT NULL,
    POSITION_NUM INT NOT NULL,
    X DECIMAL(10, 3) NOT NULL,
    Y DECIMAL(10, 3) NOT NULL,
    Z DECIMAL(10, 3) NOT NULL,
    W DECIMAL(10, 3) NOT NULL,
    PRIMARY KEY (ID),
    FOREIGN KEY (SECTION_ID) REFERENCES SECTION(ID)
);

CREATE TABLE GUIDE_HISTORY (
	ID INT NOT NULL AUTO_INCREMENT,
    START_DT DATETIME NOT NULL DEFAULT NOW(),
    END_DT DATETIME NOT NULL DEFAULT NOW(),
	PRIMARY KEY (ID)
);

CREATE TABLE HISTORY (
	ID INT NOT NULL AUTO_INCREMENT,
    ITEM_ID INT NOT NULL,
    CREATE_DT DATETIME NOT NULL DEFAULT NOW(),
    PRIMARY KEY (ID),
    FOREIGN KEY (ITEM_ID) REFERENCES ITEM(ID)
);

