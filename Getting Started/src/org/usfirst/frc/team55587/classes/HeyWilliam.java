package org.usfirst.frc.team55587.classes;
public class HeyWilliam
{
    private static double total;
    private static double sum;

    public static void main( String [] args )
    {
    	sum = 0;
        total = 0;
        heyWilliam( 0 );
    }
    
    public static void heyWilliam( int i )
    {
        if( i == 0 )
        {
            total += 4;
            System.out.println( "Hey " );
            heyWilliam( i + 3 );
        }
        else if( Math.sqrt( i ) == 4 && sum <= 6 )
        {
            total += 3;
            System.out.println( "great " );
            heyWilliam( 0 );
        }
        else if( i % 2 == 0 && i % 6 == 0 )
        {
            total -= 5;
            System.out.println( "you " );
            heyWilliam( i + 53 );
        }
        else if( i == 13 )
        {
        	total += -2;
            System.out.println( "a " );
            heyWilliam( i + 3 );
        }
        else if( Math.pow( Math.abs( sum ), 3 ) >= 216 )
        {
            total *= 50;
            System.out.println( "failure." );
        }
        else if( sum % 2 == 1 && i % 3 == 0 )
        {
        	sum += 1;
        	total += 1;
        	System.out.println( "William, " );
        	heyWilliam( 12 );
        }
        else if( Math.pow( Math.abs( total ), 2 ) >= 676 )
        {
            total /= 3;
            System.out.println( "success." );
        }
        else if( i % 13 == 0 )
        {
            total -= 1;
            System.out.println( "are " );
            heyWilliam( i / 5 );
        }
        else if( i % 3 == 0 )
        {
        	sum += 1;
            total += 1;
            System.out.println( "William, ");
            heyWilliam( i * 4 );
        }
        System.out.println( "Learn to Measure." );
    }
}