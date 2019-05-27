/**
 * 
 */
package br.insper.robot19.test;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import br.insper.robot19.GridMap;

/**
 * @author anton
 *
 */
class GridMapTest {

	/**
	 * @throws java.lang.Exception
	 */
	@BeforeAll
	static void setUpBeforeClass() throws Exception {
	}

	/**
	 * @throws java.lang.Exception
	 */
	@AfterAll
	static void tearDownAfterClass() throws Exception {
	}

	/**
	 * @throws java.lang.Exception
	 */
	@BeforeEach
	void setUp() throws Exception {
	}

	/**
	 * @throws java.lang.Exception
	 */
	@AfterEach
	void tearDown() throws Exception {
	}

	/**
	 * Test method for {@link br.insper.robot19.GridMap#fromString(java.lang.String)}.
	 */
	@Test
	void testFromString() {

		String textMap =
				".........." + System.lineSeparator() +
				".0........" + System.lineSeparator() +
				"...####..." + System.lineSeparator() +
				"...#.G...." + System.lineSeparator() +
				"...33333.." + System.lineSeparator() +
				"...XXXXX.." + System.lineSeparator() +
				".........." + System.lineSeparator();
		
		GridMap map = GridMap.fromString(textMap);
		String genTextMap = map.toString();
		
		System.out.println("Groundtruth length: " + textMap.length());
		System.out.println("Generated length: " + genTextMap.length());
		System.out.print(genTextMap);
		
		if(!textMap.equals(genTextMap))
			fail("Text parsing generates a wrong map.");
	}

	/**
	 * Test method for {@link br.insper.robot19.GridMap#drawWall(float, float, float, float)}.
	 */
	//@Test
	//void testDrawWall() {
	//	fail("Not yet implemented");
	//}

}
