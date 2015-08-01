/**
 * Created by Mayanka on 23-Jul-15.
 */
object MainClass {

  def main(args: Array[String]) {
    val sentimentAnalyzer: SentimentAnalyzer = new SentimentAnalyzer
    val tweetWithSentiment: TweetWithSentiment = sentimentAnalyzer.findSentiment("Doctor james is a very good doctor.James has good hospitality.James has more patience and talks to patients politely.James is not avilable at all time")
    System.out.println(tweetWithSentiment)
  }
}
