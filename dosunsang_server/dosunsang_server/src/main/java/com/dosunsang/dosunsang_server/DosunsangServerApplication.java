package com.dosunsang.dosunsang_server;

import lombok.extern.slf4j.Slf4j;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;


@SpringBootApplication
@Slf4j
public class DosunsangServerApplication {

	public static void main(String[] args) {
		Logger logger = LoggerFactory.getLogger(DosunsangServerApplication.class);

		SpringApplication.run(DosunsangServerApplication.class, args);
	}

}
