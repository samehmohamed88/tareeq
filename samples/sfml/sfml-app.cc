#include <SFML/Graphics.hpp>
#include <string>

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "SFML Table Example");


    const int numRows = 5;
    const int numCols = 4;
    const float cellWidth = 100.0f;
    const float cellHeight = 50.0f;
    const float startX = 100.0f; // Starting X position of the table
    const float startY = 100.0f; // Starting Y position of the table

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();

        // Draw Table Cells
        for (int i = 0; i < numRows; ++i)
        {
            for (int j = 0; j < numCols; ++j)
            {
                sf::RectangleShape cell(sf::Vector2f(cellWidth, cellHeight));
                cell.setPosition(startX + j * cellWidth, startY + i * cellHeight);
                window.draw(cell);

                // Add Text to Cell
                sf::Text text;
//                text.setFont(font);
                text.setString("Text"); // Replace with dynamic content as needed
                text.setCharacterSize(14);
                text.setFillColor(sf::Color::Black);
                text.setPosition(cell.getPosition().x + 10, cell.getPosition().y + 10);
                window.draw(text);
            }
        }

        window.display();
    }

    return 0;
}
